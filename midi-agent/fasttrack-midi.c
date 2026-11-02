// fasttrack-midi: per-user agent exposing the FastTrack Pro MIDI port to
// CoreMIDI as a virtual source + destination, bridged to fasttrackd over XPC.
#include "../common/ftp_proto.h"
#include "../common/ftp_shm.h"

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMIDI/CoreMIDI.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <stdio.h>
#include <string.h>
#include <xpc/xpc.h>

static dispatch_queue_t g_q;
static xpc_connection_t g_conn;
static MIDIClientRef g_client;
static MIDIEndpointRef g_src, g_dst;

/* USB-MIDI 1.0 event packets: byte 0 = cable << 4 | CIN, then 3 MIDI bytes. */
static int cin_length(uint8_t cin)
{
    static const int len[16] = { 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1 };
    return len[cin & 0x0f];
}

static void usb_to_midi(const uint8_t *pkts, size_t len)
{
    Byte buf[512];
    MIDIPacketList *pl = (MIDIPacketList *)buf;
    MIDIPacket *p = MIDIPacketListInit(pl);
    for (size_t i = 0; i + 3 < len; i += 4) {
        int n = cin_length(pkts[i]);
        if (!n) continue;
        p = MIDIPacketListAdd(pl, sizeof buf, p, mach_absolute_time(), (ByteCount)n, pkts + i + 1);
        if (!p) break;
    }
    if (pl->numPackets) MIDIReceived(g_src, pl);
}

/* MIDI byte stream -> USB-MIDI packets (cable 0). */
typedef struct {
    uint8_t running;      // running status
    uint8_t msg[3];
    int     have, need;
    uint8_t sysex[3];
    int     sx;
    bool    in_sysex;
} midi_parser;

static size_t emit(uint8_t *out, size_t o, uint8_t cin, const uint8_t *b, int n)
{
    out[o] = cin;
    out[o + 1] = n > 0 ? b[0] : 0;
    out[o + 2] = n > 1 ? b[1] : 0;
    out[o + 3] = n > 2 ? b[2] : 0;
    return o + 4;
}

static int data_len(uint8_t status)
{
    switch (status & 0xf0) {
    case 0xc0: case 0xd0: return 1;
    case 0xf0:
        switch (status) { case 0xf1: case 0xf3: return 1; case 0xf2: return 2; default: return 0; }
    }
    return 2;
}

static size_t midi_to_usb(midi_parser *ps, const uint8_t *in, size_t n, uint8_t *out, size_t cap)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 8 <= cap; i++) {
        uint8_t b = in[i];
        if (b >= 0xf8) { o = emit(out, o, 0x0f, &b, 1); continue; } // realtime, any time
        if (ps->in_sysex) {
            if (b == 0xf7) {
                ps->sysex[ps->sx++] = b;
                o = emit(out, o, (uint8_t)(0x04 + ps->sx), ps->sysex, ps->sx); // CIN 5/6/7: end with 1/2/3 bytes
                ps->in_sysex = false; ps->sx = 0;
                continue;
            }
            if (b & 0x80) { ps->in_sysex = false; ps->sx = 0; } // aborted, fall through
            else {
                ps->sysex[ps->sx++] = b;
                if (ps->sx == 3) { o = emit(out, o, 0x04, ps->sysex, 3); ps->sx = 0; }
                continue;
            }
        }
        if (b == 0xf0) { ps->in_sysex = true; ps->sysex[0] = b; ps->sx = 1; ps->running = 0; continue; }
        if (b & 0x80) {
            ps->msg[0] = b; ps->have = 1; ps->need = data_len(b);
            if (b < 0xf0) ps->running = b; else ps->running = 0;
            if (ps->need == 0) {
                o = emit(out, o, b >= 0xf0 ? 0x05 : (uint8_t)(b >> 4), ps->msg, 1);
                ps->have = 0;
            }
            continue;
        }
        if (ps->have == 0) { // running status
            if (!ps->running) continue;
            ps->msg[0] = ps->running; ps->have = 1; ps->need = data_len(ps->running);
        }
        ps->msg[ps->have++] = b;
        if (ps->have == ps->need + 1) {
            uint8_t st = ps->msg[0];
            uint8_t cin = st >= 0xf0 ? (ps->need == 1 ? 0x02 : 0x03) : (uint8_t)(st >> 4);
            o = emit(out, o, cin, ps->msg, ps->have);
            ps->have = 0;
        }
    }
    return o;
}

static midi_parser g_parser;

static void send_usb(const uint8_t *pkts, size_t len)
{
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(m, FTP_KEY_OP, "midi_out");
    xpc_dictionary_set_data(m, FTP_KEY_DATA, pkts, len);
    xpc_connection_send_message(g_conn, m);
    xpc_release(m);
}

static void subscribe(void)
{
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(m, FTP_KEY_OP, "subscribe_midi");
    xpc_connection_send_message(g_conn, m);
    xpc_release(m);
}

int main(void)
{
    g_q = dispatch_queue_create("be.jsix.fasttrack-midi", DISPATCH_QUEUE_SERIAL);
    if (MIDIClientCreate(CFSTR("FastTrack Pro"), NULL, NULL, &g_client)) { fprintf(stderr, "MIDIClientCreate failed\n"); return 1; }
    MIDISourceCreate(g_client, CFSTR("FastTrack Pro MIDI"), &g_src);
    MIDIDestinationCreateWithBlock(g_client, CFSTR("FastTrack Pro MIDI"), &g_dst, ^(const MIDIPacketList *pl, void *src) {
        uint8_t usb[1024];
        const MIDIPacket *p = &pl->packet[0];
        for (UInt32 i = 0; i < pl->numPackets; i++) {
            size_t n = midi_to_usb(&g_parser, p->data, p->length, usb, sizeof usb);
            if (n) send_usb(usb, n);
            p = MIDIPacketNext(p);
        }
    });
    // stable IDs so apps remember the ports
    MIDIObjectSetIntegerProperty(g_src, kMIDIPropertyUniqueID, 0x07632012);
    MIDIObjectSetIntegerProperty(g_dst, kMIDIPropertyUniqueID, 0x07632013);
    MIDIObjectSetStringProperty(g_src, kMIDIPropertyManufacturer, CFSTR("M-Audio"));
    MIDIObjectSetStringProperty(g_dst, kMIDIPropertyManufacturer, CFSTR("M-Audio"));

    g_conn = xpc_connection_create_mach_service(FTP_SERVICE_NAME, g_q, XPC_CONNECTION_MACH_SERVICE_PRIVILEGED);
    xpc_connection_set_event_handler(g_conn, ^(xpc_object_t ev) {
        if (xpc_get_type(ev) == XPC_TYPE_DICTIONARY) {
            const char *op = xpc_dictionary_get_string(ev, FTP_KEY_OP);
            if (op && !strcmp(op, "midi_in")) {
                size_t len = 0;
                const void *d = xpc_dictionary_get_data(ev, FTP_KEY_DATA, &len);
                if (d) usb_to_midi(d, len);
            }
        } else if (ev == XPC_ERROR_CONNECTION_INTERRUPTED) {
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, NSEC_PER_SEC), g_q, ^{ subscribe(); });
        }
    });
    xpc_connection_resume(g_conn);
    subscribe();
    CFRunLoopRun();
    return 0;
}
