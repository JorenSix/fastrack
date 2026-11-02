// XPC protocol between fasttrackd and its clients (HAL plug-in, MIDI agent).
// Every message is a dictionary with an "op" string.
//
//  client -> daemon (with reply):
//    "hello"        -> { shm: xpc_shmem, present: bool, rate: uint64 }
//    "start"        { rate: uint64, input: uint64 (4 analog | 5 digital) } -> { ok: bool, error: string }
//    "stop"         -> { ok }
//    "set_input"    { input: uint64 } -> { ok }
//  client -> daemon (no reply):
//    "subscribe_midi"
//    "midi_out"     { data: USB-MIDI event packets }
//  daemon -> client (events on the peer connection):
//    "status"       { present: bool }
//    "midi_in"      { data: USB-MIDI event packets }
#pragma once

#define FTP_KEY_OP      "op"
#define FTP_KEY_SHM     "shm"
#define FTP_KEY_PRESENT "present"
#define FTP_KEY_RATE    "rate"
#define FTP_KEY_INPUT   "input"
#define FTP_KEY_OK      "ok"
#define FTP_KEY_ERROR   "error"
#define FTP_KEY_DATA    "data"
