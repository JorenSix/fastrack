// FastTrackPro.driver — Core Audio AudioServerPlugIn for the M-Audio FastTrack Pro.
//
// Runs inside coreaudiod. It owns no USB: audio moves through a shared-memory
// ring served by the root daemon fasttrackd (see common/ftp_shm.h), control
// goes over XPC. Object model follows Apple's NullAudio sample:
//
//   PlugIn (1)
//    └ Device (2)            4 out / 2 in, 44.1 / 48 / 88.2 / 96 kHz
//       ├ Input stream (3)   2 ch, physical 24-bit integer
//       ├ Input volume (4), input mute (5), input source selector (6)
//       ├ Output stream (7)  4 ch, physical 24-bit integer
//       └ Output volume (8), output mute (9)
//
// The hardware has no feature units, so volume/mute are applied in software
// (ProcessMix / ProcessInput on float data).
#include "../common/ftp_shm.h"
#include "../common/ftp_proto.h"

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <math.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <xpc/xpc.h>

#define FTP_IF_IN_ANALOG  4
#define FTP_IF_IN_DIGITAL 5

enum {
    kObj_PlugIn = kAudioObjectPlugInObject,
    kObj_Device = 2,
    kObj_InStream = 3,
    kObj_InVolume = 4,
    kObj_InMute = 5,
    kObj_InSource = 6,
    kObj_OutStream = 7,
    kObj_OutVolume = 8,
    kObj_OutMute = 9,
};

#define kDeviceUID   "FastTrackPro:0763:2012"
#define kModelUID    "FastTrackPro:Model"
#define kManufacturer "M-Audio"
#define kDeviceName  "FastTrack Pro"
#define kVolMinDB    (-64.0f)
#define kVolMaxDB    (0.0f)
#define kOutCh       4
#define kInCh        2
#define kSourceAnalog 1
#define kSourceSPDIF  2

static const Float64 kRates[] = { 44100.0, 48000.0, 88200.0, 96000.0 };
#define kNumRates (sizeof kRates / sizeof kRates[0])

static os_log_t g_log;
#define LOG(...) os_log(g_log, __VA_ARGS__)

/* ----------------------------------------------------------------- state */

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static AudioServerPlugInHostRef g_host;
static UInt32 g_refcount;

static Float64 g_rate = 48000.0;
static UInt32  g_io_count;
static UInt32  g_source = kSourceAnalog;
static _Atomic float g_out_scalar = 1.0f, g_in_scalar = 1.0f;
static _Atomic bool  g_out_mute, g_in_mute;

static dispatch_queue_t g_xq;      // XPC connection target queue (events, async replies)
static dispatch_queue_t g_rq;      // synchronous requests (never the target queue)
static xpc_connection_t g_conn;
static ftp_shm *_Atomic g_shm;
static _Atomic bool g_present;

static Float64  g_ticks_per_sample_nominal;

/* -------------------------------------------------------------- helpers */

static float scalar_to_db(float s)
{
    if (s <= 0.0f) return kVolMinDB;
    if (s >= 1.0f) return kVolMaxDB;
    return kVolMinDB + (kVolMaxDB - kVolMinDB) * s;
}

static float db_to_scalar(float db)
{
    if (db <= kVolMinDB) return 0.0f;
    if (db >= kVolMaxDB) return 1.0f;
    return (db - kVolMinDB) / (kVolMaxDB - kVolMinDB);
}

static float scalar_to_gain(float s)
{
    if (s <= 0.0f) return 0.0f;
    return powf(10.0f, scalar_to_db(s) / 20.0f);
}

static void notify(AudioObjectID obj, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope)
{
    AudioObjectPropertyAddress a = { sel, scope, kAudioObjectPropertyElementMain };
    if (g_host) g_host->PropertiesChanged(g_host, obj, 1, &a);
}

static void store_settings(void)
{
    if (!g_host) return;
    float vals[2] = { atomic_load(&g_out_scalar), atomic_load(&g_in_scalar) };
    int flags = (atomic_load(&g_out_mute) ? 1 : 0) | (atomic_load(&g_in_mute) ? 2 : 0) | (g_source == kSourceSPDIF ? 4 : 0);
    CFMutableDictionaryRef d = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFNumberRef ov = CFNumberCreate(NULL, kCFNumberFloatType, &vals[0]);
    CFNumberRef iv = CFNumberCreate(NULL, kCFNumberFloatType, &vals[1]);
    CFNumberRef fl = CFNumberCreate(NULL, kCFNumberIntType, &flags);
    CFNumberRef rt = CFNumberCreate(NULL, kCFNumberFloat64Type, &g_rate);
    CFDictionarySetValue(d, CFSTR("out_volume"), ov);
    CFDictionarySetValue(d, CFSTR("in_volume"), iv);
    CFDictionarySetValue(d, CFSTR("flags"), fl);
    CFDictionarySetValue(d, CFSTR("rate"), rt);
    g_host->WriteToStorage(g_host, CFSTR("settings"), d);
    CFRelease(ov); CFRelease(iv); CFRelease(fl); CFRelease(rt); CFRelease(d);
}

static void load_settings(void)
{
    CFPropertyListRef p = NULL;
    if (g_host->CopyFromStorage(g_host, CFSTR("settings"), &p) != 0 || !p) return;
    if (CFGetTypeID(p) == CFDictionaryGetTypeID()) {
        float f;
        int flags;
        Float64 r;
        CFNumberRef n;
        if ((n = CFDictionaryGetValue(p, CFSTR("out_volume"))) && CFNumberGetValue(n, kCFNumberFloatType, &f)) atomic_store(&g_out_scalar, f);
        if ((n = CFDictionaryGetValue(p, CFSTR("in_volume"))) && CFNumberGetValue(n, kCFNumberFloatType, &f)) atomic_store(&g_in_scalar, f);
        if ((n = CFDictionaryGetValue(p, CFSTR("flags"))) && CFNumberGetValue(n, kCFNumberIntType, &flags)) {
            atomic_store(&g_out_mute, (flags & 1) != 0);
            atomic_store(&g_in_mute, (flags & 2) != 0);
            g_source = (flags & 4) ? kSourceSPDIF : kSourceAnalog;
        }
        if ((n = CFDictionaryGetValue(p, CFSTR("rate"))) && CFNumberGetValue(n, kCFNumberFloat64Type, &r))
            for (size_t i = 0; i < kNumRates; i++) if (kRates[i] == r) g_rate = r;
    }
    CFRelease(p);
}

/* ------------------------------------------------------------------ XPC */

static void set_present(bool present)
{
    if (atomic_exchange(&g_present, present) == present) return;
    LOG("FastTrack Pro %{public}s", present ? "connected" : "disconnected");
    AudioObjectPropertyAddress a[2] = {
        { kAudioPlugInPropertyDeviceList, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
    };
    if (!present) notify(kObj_Device, kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal);
    if (g_host) g_host->PropertiesChanged(g_host, kObj_PlugIn, 2, a);
}

static void hello(void);

static void connect_daemon(void)
{
    g_conn = xpc_connection_create_mach_service(FTP_SERVICE_NAME, g_xq, XPC_CONNECTION_MACH_SERVICE_PRIVILEGED);
    xpc_connection_set_event_handler(g_conn, ^(xpc_object_t ev) {
        if (xpc_get_type(ev) == XPC_TYPE_DICTIONARY) {
            const char *op = xpc_dictionary_get_string(ev, FTP_KEY_OP);
            if (op && !strcmp(op, "status")) set_present(xpc_dictionary_get_bool(ev, FTP_KEY_PRESENT));
        } else if (ev == XPC_ERROR_CONNECTION_INTERRUPTED) {
            // daemon restarted: re-handshake (new shared memory)
            set_present(false);
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, NSEC_PER_SEC), g_xq, ^{ hello(); });
        }
    });
    xpc_connection_resume(g_conn);
}

static void hello(void)
{
    xpc_object_t msg = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(msg, FTP_KEY_OP, "hello");
    xpc_connection_send_message_with_reply(g_conn, msg, g_xq, ^(xpc_object_t r) {
        if (xpc_get_type(r) != XPC_TYPE_DICTIONARY) {
            LOG("fasttrackd not reachable, retrying");
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 3 * NSEC_PER_SEC), g_xq, ^{ hello(); });
            return;
        }
        xpc_object_t shm = xpc_dictionary_get_value(r, FTP_KEY_SHM);
        void *addr = NULL;
        size_t len = shm ? xpc_shmem_map(shm, &addr) : 0;
        if (len >= sizeof(ftp_shm) && ((ftp_shm *)addr)->magic == FTP_SHM_MAGIC && ((ftp_shm *)addr)->version == FTP_SHM_VERSION) {
            // An old mapping (from a previous daemon instance) is intentionally
            // leaked: the IO thread may still be touching it.
            atomic_store(&g_shm, (ftp_shm *)addr);
            set_present(xpc_dictionary_get_bool(r, FTP_KEY_PRESENT));
        } else {
            LOG("fasttrackd shared memory mismatch");
        }
    });
    xpc_release(msg);
}

static xpc_object_t request(const char *op, uint64_t rate, uint64_t input)
{
    xpc_object_t msg = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(msg, FTP_KEY_OP, op);
    xpc_dictionary_set_uint64(msg, FTP_KEY_RATE, rate);
    xpc_dictionary_set_uint64(msg, FTP_KEY_INPUT, input);
    xpc_object_t r = xpc_connection_send_message_with_reply_sync(g_conn, msg);
    xpc_release(msg);
    return r;
}

static uint64_t input_iface(void) { return g_source == kSourceSPDIF ? FTP_IF_IN_DIGITAL : FTP_IF_IN_ANALOG; }

/* ---------------------------------------------------------- formats etc */

static UInt32 out_queue_frames(void)
{
    ftp_shm *s = atomic_load(&g_shm);
    UInt32 q = s ? atomic_load(&s->output_queue_frames) : 0;
    return q ? q : (UInt32)(g_rate * 0.016);
}

static UInt32 safety_out(void) { return (UInt32)(g_rate * 0.003); }   // DLL jitter margin
static UInt32 safety_in(void)  { return (UInt32)(g_rate * 0.006); }   // 2 ms transfers + jitter

static void fill_physical(AudioStreamBasicDescription *f, UInt32 ch, Float64 rate)
{
    memset(f, 0, sizeof *f);
    f->mSampleRate = rate;
    f->mFormatID = kAudioFormatLinearPCM;
    f->mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    f->mBitsPerChannel = 24;
    f->mChannelsPerFrame = ch;
    f->mBytesPerFrame = 3 * ch;
    f->mFramesPerPacket = 1;
    f->mBytesPerPacket = 3 * ch;
}

static void fill_virtual(AudioStreamBasicDescription *f, UInt32 ch, Float64 rate)
{
    memset(f, 0, sizeof *f);
    f->mSampleRate = rate;
    f->mFormatID = kAudioFormatLinearPCM;
    f->mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
    f->mBitsPerChannel = 32;
    f->mChannelsPerFrame = ch;
    f->mBytesPerFrame = 4 * ch;
    f->mFramesPerPacket = 1;
    f->mBytesPerPacket = 4 * ch;
}

static bool is_control(AudioObjectID o) { return o == kObj_InVolume || o == kObj_InMute || o == kObj_InSource || o == kObj_OutVolume || o == kObj_OutMute; }
static bool is_stream(AudioObjectID o) { return o == kObj_InStream || o == kObj_OutStream; }
static bool object_exists(AudioObjectID o)
{
    if (o == kObj_PlugIn) return true;
    if (!atomic_load(&g_present)) return false;
    return o == kObj_Device || is_stream(o) || is_control(o);
}

/* ------------------------------------------------------ COM boilerplate */

static HRESULT QueryInterface(void *self, REFIID iid, LPVOID *out);
static ULONG AddRef(void *self);
static ULONG Release(void *self);
static OSStatus Initialize(AudioServerPlugInDriverRef d, AudioServerPlugInHostRef host);
static OSStatus CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc, const AudioServerPlugInClientInfo *ci, AudioObjectID *out) { return kAudioHardwareUnsupportedOperationError; }
static OSStatus DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID dev) { return kAudioHardwareUnsupportedOperationError; }
static OSStatus AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID dev, const AudioServerPlugInClientInfo *ci) { return 0; }
static OSStatus RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID dev, const AudioServerPlugInClientInfo *ci) { return 0; }
static OSStatus PerformDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt64 action, void *info);
static OSStatus AbortDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt64 action, void *info) { return 0; }
static Boolean HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a);
static OSStatus IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, Boolean *out);
static OSStatus GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 *out);
static OSStatus GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out);
static OSStatus SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, const void *data);
static OSStatus StartIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client);
static OSStatus StopIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client);
static OSStatus GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, Float64 *st, UInt64 *ht, UInt64 *seed);
static OSStatus WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, Boolean *will, Boolean *inPlace);
static OSStatus BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci) { return 0; }
static OSStatus DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, AudioObjectID stream, UInt32 client, UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci, void *main, void *secondary);
static OSStatus EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci) { return 0; }

static AudioServerPlugInDriverInterface g_vtable = {
    NULL, QueryInterface, AddRef, Release, Initialize, CreateDevice, DestroyDevice,
    AddDeviceClient, RemoveDeviceClient, PerformDeviceConfigurationChange, AbortDeviceConfigurationChange,
    HasProperty, IsPropertySettable, GetPropertyDataSize, GetPropertyData, SetPropertyData,
    StartIO, StopIO, GetZeroTimeStamp, WillDoIOOperation, BeginIOOperation, DoIOOperation, EndIOOperation,
};
static AudioServerPlugInDriverInterface *g_vtable_ptr = &g_vtable;
static AudioServerPlugInDriverRef g_driver = &g_vtable_ptr;

__attribute__((visibility("default")))
void *FastTrackPro_Create(CFAllocatorRef alloc, CFUUIDRef type)
{
    if (!CFEqual(type, kAudioServerPlugInTypeUUID)) return NULL;
    return g_driver;
}

static HRESULT QueryInterface(void *self, REFIID iid, LPVOID *out)
{
    if (self != g_driver || !out) return kAudioHardwareBadObjectError;
    CFUUIDRef req = CFUUIDCreateFromUUIDBytes(NULL, iid);
    bool ok = CFEqual(req, IUnknownUUID) || CFEqual(req, kAudioServerPlugInDriverInterfaceUUID);
    CFRelease(req);
    if (!ok) { *out = NULL; return E_NOINTERFACE; }
    pthread_mutex_lock(&g_lock);
    g_refcount++;
    pthread_mutex_unlock(&g_lock);
    *out = g_driver;
    return S_OK;
}

static ULONG AddRef(void *self)
{
    pthread_mutex_lock(&g_lock);
    ULONG r = ++g_refcount;
    pthread_mutex_unlock(&g_lock);
    return r;
}

static ULONG Release(void *self)
{
    pthread_mutex_lock(&g_lock);
    ULONG r = g_refcount ? --g_refcount : 0;
    pthread_mutex_unlock(&g_lock);
    return r;
}

static OSStatus Initialize(AudioServerPlugInDriverRef d, AudioServerPlugInHostRef host)
{
    g_log = os_log_create("be.jsix.fasttrack", "plugin");
    g_host = host;
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    g_ticks_per_sample_nominal = 1e9 * tb.denom / tb.numer;
    load_settings();
    g_xq = dispatch_queue_create("be.jsix.fasttrack.xpc", DISPATCH_QUEUE_SERIAL);
    g_rq = dispatch_queue_create("be.jsix.fasttrack.req", DISPATCH_QUEUE_SERIAL);
    dispatch_sync(g_xq, ^{ connect_daemon(); hello(); });
    LOG("FastTrack Pro plug-in initialized");
    return 0;
}

/* ------------------------------------------------------------ properties */

static Boolean HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a)
{
    if (!object_exists(o)) return false;
    UInt32 sz;
    return GetPropertyDataSize(d, o, pid, a, 0, NULL, &sz) == 0;
}

static OSStatus IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, Boolean *out)
{
    if (!object_exists(o)) return kAudioHardwareBadObjectError;
    *out = false;
    switch (a->mSelector) {
    case kAudioDevicePropertyNominalSampleRate: *out = o == kObj_Device; break;
    case kAudioStreamPropertyVirtualFormat:
    case kAudioStreamPropertyPhysicalFormat: *out = is_stream(o); break;
    case kAudioLevelControlPropertyScalarValue:
    case kAudioLevelControlPropertyDecibelValue: *out = o == kObj_InVolume || o == kObj_OutVolume; break;
    case kAudioBooleanControlPropertyValue: *out = o == kObj_InMute || o == kObj_OutMute; break;
    case kAudioSelectorControlPropertyCurrentItem: *out = o == kObj_InSource; break;
    }
    return 0;
}

// Writes a CFString / scalar into the caller buffer.
#define PUT(type, val) do { if (size < sizeof(type)) return kAudioHardwareBadPropertySizeError; *(type *)out = (val); *outSize = sizeof(type); return 0; } while (0)
#define PUTSTR(cstr) PUT(CFStringRef, CFStringCreateWithCString(NULL, (cstr), kCFStringEncodingUTF8))

static OSStatus put_ids(const AudioObjectID *ids, UInt32 n, UInt32 size, UInt32 *outSize, void *out)
{
    UInt32 max = size / sizeof(AudioObjectID);
    if (n > max) n = max;
    memcpy(out, ids, n * sizeof(AudioObjectID));
    *outSize = n * sizeof(AudioObjectID);
    return 0;
}

static OSStatus GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 *out)
{
    // Compute the size by asking GetPropertyData with a large scratch buffer.
    union { char buf[1024]; double align; } scratch;
    UInt32 got = 0;
    OSStatus r = GetPropertyData(d, o, pid, a, qs, q, sizeof scratch.buf, &got, scratch.buf);
    if (r) return r;
    // release any CF objects we just created
    switch (a->mSelector) {
    case kAudioObjectPropertyName: case kAudioObjectPropertyManufacturer: case kAudioDevicePropertyDeviceUID:
    case kAudioDevicePropertyModelUID: case kAudioObjectPropertyElementName: case kAudioPlugInPropertyResourceBundle:
    case kAudioSelectorControlPropertyItemName: case kAudioDevicePropertyConfigurationApplication:
        if (got == sizeof(CFStringRef) && *(CFStringRef *)scratch.buf) CFRelease(*(CFStringRef *)scratch.buf);
        break;
    }
    *out = got;
    return 0;
}

static UInt32 n_owned_device(AudioObjectID *ids)
{
    AudioObjectID all[] = { kObj_InStream, kObj_OutStream, kObj_InVolume, kObj_InMute, kObj_InSource, kObj_OutVolume, kObj_OutMute };
    memcpy(ids, all, sizeof all);
    return sizeof all / sizeof all[0];
}

static OSStatus get_plugin(const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out)
{
    bool present = atomic_load(&g_present);
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
    case kAudioObjectPropertyClass: PUT(AudioClassID, kAudioPlugInClassID);
    case kAudioObjectPropertyOwner: PUT(AudioObjectID, kAudioObjectUnknown);
    case kAudioObjectPropertyManufacturer: PUTSTR("FastTrack Pro user-space driver");
    case kAudioObjectPropertyOwnedObjects:
    case kAudioPlugInPropertyDeviceList: {
        AudioObjectID dev = kObj_Device;
        return put_ids(&dev, present ? 1 : 0, size, outSize, out);
    }
    case kAudioPlugInPropertyTranslateUIDToDevice: {
        if (qs < sizeof(CFStringRef)) return kAudioHardwareBadPropertySizeError;
        CFStringRef uid = *(CFStringRef *)q;
        AudioObjectID r = kAudioObjectUnknown;
        if (present && uid && CFStringCompare(uid, CFSTR(kDeviceUID), 0) == kCFCompareEqualTo) r = kObj_Device;
        PUT(AudioObjectID, r);
    }
    case kAudioPlugInPropertyResourceBundle: PUTSTR("");
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_device(const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out)
{
    AudioObjectPropertyScope sc = a->mScope;
    bool in = sc == kAudioObjectPropertyScopeInput, outp = sc == kAudioObjectPropertyScopeOutput;
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
    case kAudioObjectPropertyClass: PUT(AudioClassID, kAudioDeviceClassID);
    case kAudioObjectPropertyOwner: PUT(AudioObjectID, kObj_PlugIn);
    case kAudioObjectPropertyName: PUTSTR(kDeviceName);
    case kAudioObjectPropertyManufacturer: PUTSTR(kManufacturer);
    case kAudioDevicePropertyDeviceUID: PUTSTR(kDeviceUID);
    case kAudioDevicePropertyModelUID: PUTSTR(kModelUID);
    case kAudioObjectPropertyElementName: {
        static const char *outs[] = { "Out 1", "Out 2", "Out 3", "Out 4" };
        static const char *ins[] = { "In 1", "In 2" };
        UInt32 e = a->mElement;
        if (outp && e >= 1 && e <= 4) PUTSTR(outs[e - 1]);
        if (in && e >= 1 && e <= 2) PUTSTR(g_source == kSourceSPDIF ? (e == 1 ? "S/PDIF L" : "S/PDIF R") : ins[e - 1]);
        return kAudioHardwareUnknownPropertyError;
    }
    case kAudioObjectPropertyOwnedObjects: {
        AudioObjectID ids[8];
        UInt32 n = n_owned_device(ids);
        if (in) { AudioObjectID i[] = { kObj_InStream, kObj_InVolume, kObj_InMute, kObj_InSource }; return put_ids(i, 4, size, outSize, out); }
        if (outp) { AudioObjectID i[] = { kObj_OutStream, kObj_OutVolume, kObj_OutMute }; return put_ids(i, 3, size, outSize, out); }
        return put_ids(ids, n, size, outSize, out);
    }
    case kAudioObjectPropertyControlList: {
        AudioObjectID ids[] = { kObj_InVolume, kObj_InMute, kObj_InSource, kObj_OutVolume, kObj_OutMute };
        return put_ids(ids, 5, size, outSize, out);
    }
    case kAudioDevicePropertyStreams: {
        AudioObjectID ids[2]; UInt32 n = 0;
        if (!outp) ids[n++] = kObj_InStream;
        if (!in) ids[n++] = kObj_OutStream;
        return put_ids(ids, n, size, outSize, out);
    }
    case kAudioDevicePropertyTransportType: PUT(UInt32, kAudioDeviceTransportTypeUSB);
    case kAudioDevicePropertyRelatedDevices: { AudioObjectID i = kObj_Device; return put_ids(&i, 1, size, outSize, out); }
    case kAudioDevicePropertyClockDomain: PUT(UInt32, 0x0763);
    case kAudioDevicePropertyDeviceIsAlive: PUT(UInt32, atomic_load(&g_present) ? 1 : 0);
    case kAudioDevicePropertyDeviceIsRunning: { pthread_mutex_lock(&g_lock); UInt32 r = g_io_count > 0; pthread_mutex_unlock(&g_lock); PUT(UInt32, r); }
    case kAudioDevicePropertyDeviceCanBeDefaultDevice: PUT(UInt32, 1);
    case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice: PUT(UInt32, 1);
    case kAudioDevicePropertyLatency: PUT(UInt32, outp ? out_queue_frames() + (UInt32)(g_rate * 0.001) : (UInt32)(g_rate * 0.002));
    case kAudioDevicePropertySafetyOffset: PUT(UInt32, outp ? safety_out() : safety_in());
    case kAudioDevicePropertyNominalSampleRate: PUT(Float64, g_rate);
    case kAudioDevicePropertyAvailableNominalSampleRates: {
        UInt32 max = size / sizeof(AudioValueRange), n = 0;
        AudioValueRange *r = out;
        for (size_t i = 0; i < kNumRates && n < max; i++, n++) { r[n].mMinimum = kRates[i]; r[n].mMaximum = kRates[i]; }
        *outSize = n * sizeof(AudioValueRange);
        return 0;
    }
    case kAudioDevicePropertyIsHidden: PUT(UInt32, 0);
    case kAudioDevicePropertyZeroTimeStampPeriod: PUT(UInt32, FTP_RING_FRAMES);
    case kAudioDevicePropertyClockIsStable: PUT(UInt32, 1);
    case kAudioDevicePropertyClockAlgorithm: PUT(UInt32, kAudioDeviceClockAlgorithmSimpleIIR);
    case kAudioDevicePropertyPreferredChannelsForStereo: {
        if (size < 2 * sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        ((UInt32 *)out)[0] = 1; ((UInt32 *)out)[1] = 2;
        *outSize = 2 * sizeof(UInt32);
        return 0;
    }
    case kAudioDevicePropertyPreferredChannelLayout: {
        UInt32 ch = in ? kInCh : kOutCh;
        UInt32 need = (UInt32)(offsetof(AudioChannelLayout, mChannelDescriptions) + ch * sizeof(AudioChannelDescription));
        if (size < need) return kAudioHardwareBadPropertySizeError;
        AudioChannelLayout *l = out;
        memset(l, 0, need);
        l->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
        l->mNumberChannelDescriptions = ch;
        for (UInt32 i = 0; i < ch; i++)
            l->mChannelDescriptions[i].mChannelLabel = i == 0 ? kAudioChannelLabel_Left : i == 1 ? kAudioChannelLabel_Right : kAudioChannelLabel_Discrete_2 + (i - 2);
        *outSize = need;
        return 0;
    }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_stream(AudioObjectID o, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out)
{
    bool in = o == kObj_InStream;
    UInt32 ch = in ? kInCh : kOutCh;
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
    case kAudioObjectPropertyClass: PUT(AudioClassID, kAudioStreamClassID);
    case kAudioObjectPropertyOwner: PUT(AudioObjectID, kObj_Device);
    case kAudioObjectPropertyName: PUTSTR(in ? "FastTrack Pro In" : "FastTrack Pro Out");
    case kAudioObjectPropertyOwnedObjects: *outSize = 0; return 0;
    case kAudioStreamPropertyIsActive: PUT(UInt32, 1);
    case kAudioStreamPropertyDirection: PUT(UInt32, in ? 1 : 0);
    case kAudioStreamPropertyTerminalType: PUT(UInt32, in ? (g_source == kSourceSPDIF ? kAudioStreamTerminalTypeDigitalAudioInterface : kAudioStreamTerminalTypeLine)
                                                         : kAudioStreamTerminalTypeLine);
    case kAudioStreamPropertyStartingChannel: PUT(UInt32, 1);
    case kAudioStreamPropertyLatency: PUT(UInt32, 0);
    case kAudioStreamPropertyVirtualFormat: {
        AudioStreamBasicDescription f; fill_virtual(&f, ch, g_rate); PUT(AudioStreamBasicDescription, f);
    }
    case kAudioStreamPropertyPhysicalFormat: {
        AudioStreamBasicDescription f; fill_physical(&f, ch, g_rate); PUT(AudioStreamBasicDescription, f);
    }
    case kAudioStreamPropertyAvailableVirtualFormats:
    case kAudioStreamPropertyAvailablePhysicalFormats: {
        UInt32 max = size / sizeof(AudioStreamRangedDescription), n = 0;
        AudioStreamRangedDescription *r = out;
        for (size_t i = 0; i < kNumRates && n < max; i++, n++) {
            if (a->mSelector == kAudioStreamPropertyAvailableVirtualFormats) fill_virtual(&r[n].mFormat, ch, kRates[i]);
            else fill_physical(&r[n].mFormat, ch, kRates[i]);
            r[n].mSampleRateRange.mMinimum = r[n].mSampleRateRange.mMaximum = kRates[i];
        }
        *outSize = n * sizeof(AudioStreamRangedDescription);
        return 0;
    }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_control(AudioObjectID o, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out)
{
    bool in = o == kObj_InVolume || o == kObj_InMute || o == kObj_InSource;
    bool vol = o == kObj_InVolume || o == kObj_OutVolume;
    bool mute = o == kObj_InMute || o == kObj_OutMute;
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass:
        PUT(AudioClassID, vol ? kAudioLevelControlClassID : mute ? kAudioBooleanControlClassID : kAudioSelectorControlClassID);
    case kAudioObjectPropertyClass:
        PUT(AudioClassID, vol ? kAudioVolumeControlClassID : mute ? kAudioMuteControlClassID : kAudioDataSourceControlClassID);
    case kAudioObjectPropertyOwner: PUT(AudioObjectID, kObj_Device);
    case kAudioObjectPropertyOwnedObjects: *outSize = 0; return 0;
    case kAudioObjectPropertyName: PUTSTR(vol ? (in ? "Input Volume" : "Output Volume") : mute ? (in ? "Input Mute" : "Output Mute") : "Input Source");
    case kAudioControlPropertyScope: PUT(AudioObjectPropertyScope, in ? kAudioObjectPropertyScopeInput : kAudioObjectPropertyScopeOutput);
    case kAudioControlPropertyElement: PUT(AudioObjectPropertyElement, kAudioObjectPropertyElementMain);
    }
    if (vol) {
        float s = atomic_load(o == kObj_InVolume ? &g_in_scalar : &g_out_scalar);
        switch (a->mSelector) {
        case kAudioLevelControlPropertyScalarValue: PUT(Float32, s);
        case kAudioLevelControlPropertyDecibelValue: PUT(Float32, scalar_to_db(s));
        case kAudioLevelControlPropertyDecibelRange: { AudioValueRange r = { kVolMinDB, kVolMaxDB }; PUT(AudioValueRange, r); }
        case kAudioLevelControlPropertyConvertScalarToDecibels:
            if (size < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            *(Float32 *)out = scalar_to_db(*(Float32 *)out); *outSize = sizeof(Float32); return 0;
        case kAudioLevelControlPropertyConvertDecibelsToScalar:
            if (size < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            *(Float32 *)out = db_to_scalar(*(Float32 *)out); *outSize = sizeof(Float32); return 0;
        }
    }
    if (mute && a->mSelector == kAudioBooleanControlPropertyValue)
        PUT(UInt32, atomic_load(o == kObj_InMute ? &g_in_mute : &g_out_mute) ? 1 : 0);
    if (o == kObj_InSource) {
        switch (a->mSelector) {
        case kAudioSelectorControlPropertyCurrentItem: PUT(UInt32, g_source);
        case kAudioSelectorControlPropertyAvailableItems: {
            UInt32 items[] = { kSourceAnalog, kSourceSPDIF };
            UInt32 n = size / sizeof(UInt32); if (n > 2) n = 2;
            memcpy(out, items, n * sizeof(UInt32));
            *outSize = n * sizeof(UInt32);
            return 0;
        }
        case kAudioSelectorControlPropertyItemName: {
            if (qs < sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
            UInt32 item = *(const UInt32 *)q;
            if (item == kSourceAnalog) PUTSTR("Analog In 1/2");
            if (item == kSourceSPDIF) PUTSTR("S/PDIF In");
            return kAudioHardwareIllegalOperationError;
        }
        case kAudioSelectorControlPropertyItemKind: PUT(UInt32, 0);
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, UInt32 *outSize, void *out)
{
    if (!object_exists(o)) return kAudioHardwareBadObjectError;
    if (o == kObj_PlugIn) return get_plugin(a, qs, q, size, outSize, out);
    if (o == kObj_Device) return get_device(a, qs, q, size, outSize, out);
    if (is_stream(o)) return get_stream(o, a, qs, q, size, outSize, out);
    return get_control(o, a, qs, q, size, outSize, out);
}

static OSStatus SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 size, const void *data)
{
    if (!object_exists(o)) return kAudioHardwareBadObjectError;
    switch (a->mSelector) {
    case kAudioDevicePropertyNominalSampleRate: {
        if (o != kObj_Device) break;
        if (size < sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
        Float64 r = *(const Float64 *)data;
        bool ok = false;
        for (size_t i = 0; i < kNumRates; i++) if (kRates[i] == r) ok = true;
        if (!ok) return kAudioHardwareIllegalOperationError;
        if (r != g_rate) g_host->RequestDeviceConfigurationChange(g_host, kObj_Device, (UInt64)r, NULL);
        return 0;
    }
    case kAudioStreamPropertyVirtualFormat:
    case kAudioStreamPropertyPhysicalFormat: {
        if (!is_stream(o)) break;
        if (size < sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
        const AudioStreamBasicDescription *f = data;
        bool ok = false;
        for (size_t i = 0; i < kNumRates; i++) if (kRates[i] == f->mSampleRate) ok = true;
        if (!ok) return kAudioDeviceUnsupportedFormatError;
        if (f->mSampleRate != g_rate) g_host->RequestDeviceConfigurationChange(g_host, kObj_Device, (UInt64)f->mSampleRate, NULL);
        return 0;
    }
    case kAudioLevelControlPropertyScalarValue:
    case kAudioLevelControlPropertyDecibelValue: {
        if (o != kObj_InVolume && o != kObj_OutVolume) break;
        if (size < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
        float v = *(const Float32 *)data;
        float s = a->mSelector == kAudioLevelControlPropertyScalarValue ? fminf(fmaxf(v, 0.0f), 1.0f) : db_to_scalar(v);
        _Atomic float *p = o == kObj_InVolume ? &g_in_scalar : &g_out_scalar;
        if (atomic_exchange(p, s) != s) {
            AudioObjectPropertyAddress ch[2] = {
                { kAudioLevelControlPropertyScalarValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
                { kAudioLevelControlPropertyDecibelValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
            };
            g_host->PropertiesChanged(g_host, o, 2, ch);
            store_settings();
        }
        return 0;
    }
    case kAudioBooleanControlPropertyValue: {
        if (o != kObj_InMute && o != kObj_OutMute) break;
        if (size < sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        bool v = *(const UInt32 *)data != 0;
        if (atomic_exchange(o == kObj_InMute ? &g_in_mute : &g_out_mute, v) != v) {
            notify(o, kAudioBooleanControlPropertyValue, kAudioObjectPropertyScopeGlobal);
            store_settings();
        }
        return 0;
    }
    case kAudioSelectorControlPropertyCurrentItem: {
        if (o != kObj_InSource) break;
        if (size < sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        UInt32 v = *(const UInt32 *)data;
        if (v != kSourceAnalog && v != kSourceSPDIF) return kAudioHardwareIllegalOperationError;
        if (v != g_source) {
            g_source = v;
            uint64_t iface = input_iface();
            dispatch_async(g_rq, ^{ xpc_object_t r = request("set_input", 0, iface); if (r) xpc_release(r); });
            notify(o, kAudioSelectorControlPropertyCurrentItem, kAudioObjectPropertyScopeGlobal);
            notify(kObj_InStream, kAudioStreamPropertyTerminalType, kAudioObjectPropertyScopeGlobal);
            store_settings();
        }
        return 0;
    }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus PerformDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt64 action, void *info)
{
    Float64 r = (Float64)action;
    bool ok = false;
    for (size_t i = 0; i < kNumRates; i++) if (kRates[i] == r) ok = true;
    if (!ok) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&g_lock);
    g_rate = r;
    bool running = g_io_count > 0;
    pthread_mutex_unlock(&g_lock);
    if (running) {
        uint64_t rate = (uint64_t)r, iface = input_iface();
        dispatch_sync(g_rq, ^{ xpc_object_t x = request("start", rate, iface); if (x) xpc_release(x); });
    }
    store_settings();
    LOG("sample rate -> %.0f Hz", r);
    return 0;
}

/* -------------------------------------------------------------------- IO */

static OSStatus StartIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client)
{
    if (dev != kObj_Device) return kAudioHardwareBadObjectError;
    if (!atomic_load(&g_present) || !atomic_load(&g_shm)) return kAudioHardwareNotRunningError;
    pthread_mutex_lock(&g_lock);
    bool first = g_io_count++ == 0;
    pthread_mutex_unlock(&g_lock);
    if (!first) return 0;

    ftp_shm *s = atomic_load(&g_shm);
    uint64_t gen_before = atomic_load(&s->generation);
    __block bool ok = false;
    __block struct { char s[128]; } err = { "" };
    uint64_t rate = (uint64_t)g_rate, iface = input_iface();
    dispatch_sync(g_rq, ^{
        xpc_object_t r = request("start", rate, iface);
        if (r && xpc_get_type(r) == XPC_TYPE_DICTIONARY) {
            ok = xpc_dictionary_get_bool(r, FTP_KEY_OK);
            const char *e = xpc_dictionary_get_string(r, FTP_KEY_ERROR);
            if (e) strlcpy(err.s, e, sizeof err.s);
        }
        if (r) xpc_release(r);
    });
    if (!ok || atomic_load(&s->generation) == gen_before) {
        LOG("StartIO failed: %{public}s", err.s[0] ? err.s : "no reply from fasttrackd");
        pthread_mutex_lock(&g_lock);
        g_io_count--;
        pthread_mutex_unlock(&g_lock);
        return kAudioHardwareNotRunningError;
    }
    notify(kObj_Device, kAudioDevicePropertyDeviceIsRunning, kAudioObjectPropertyScopeGlobal);
    return 0;
}

static OSStatus StopIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client)
{
    if (dev != kObj_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&g_lock);
    bool last = g_io_count > 0 && --g_io_count == 0;
    pthread_mutex_unlock(&g_lock);
    if (!last) return 0;
    dispatch_sync(g_rq, ^{ xpc_object_t r = request("stop", 0, 0); if (r) xpc_release(r); });
    notify(kObj_Device, kAudioDevicePropertyDeviceIsRunning, kAudioObjectPropertyScopeGlobal);
    return 0;
}

static OSStatus GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, Float64 *st, UInt64 *ht, UInt64 *seed)
{
    ftp_shm *s = atomic_load(&g_shm);
    uint64_t as, ah;
    double tps;
    if (!s || !atomic_load(&s->running) || !ftp_shm_read_clock(s, &as, &ah, &tps)) {
        // Not streaming (yet): free-run on the host clock so the HAL stays sane.
        double nominal = g_ticks_per_sample_nominal / g_rate;
        uint64_t now = mach_absolute_time();
        double cur = (double)now / nominal;
        double z = floor(cur / FTP_RING_FRAMES) * FTP_RING_FRAMES;
        *st = z;
        *ht = (UInt64)(z * nominal);
        *seed = 1;
        return 0;
    }
    uint64_t now = mach_absolute_time();
    double cur = (double)as + ((double)now - (double)ah) / tps;
    double z = floor(cur / FTP_RING_FRAMES) * FTP_RING_FRAMES;
    *st = z;
    *ht = (UInt64)((double)ah + (z - (double)as) * tps);
    *seed = atomic_load(&s->generation) + 1;
    return 0;
}

static OSStatus WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, Boolean *will, Boolean *inPlace)
{
    switch (op) {
    case kAudioServerPlugInIOOperationReadInput:
    case kAudioServerPlugInIOOperationProcessInput:
    case kAudioServerPlugInIOOperationProcessMix:
    case kAudioServerPlugInIOOperationWriteMix:
        *will = true; *inPlace = true; break;
    default:
        *will = false; *inPlace = true;
    }
    return 0;
}

static void apply_gain(float *buf, UInt32 samples, float scalar, bool mute)
{
    if (mute) { memset(buf, 0, samples * sizeof(float)); return; }
    if (scalar >= 1.0f) return;
    float g = scalar_to_gain(scalar);
    for (UInt32 i = 0; i < samples; i++) buf[i] *= g;
}

static OSStatus DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, AudioObjectID stream, UInt32 client, UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci, void *main, void *secondary)
{
    ftp_shm *s = atomic_load(&g_shm);
    const uint32_t mask = FTP_RING_FRAMES - 1;

    switch (op) {
    case kAudioServerPlugInIOOperationProcessMix:
        apply_gain(main, frames * kOutCh, atomic_load(&g_out_scalar), atomic_load(&g_out_mute));
        return 0;
    case kAudioServerPlugInIOOperationProcessInput:
        apply_gain(main, frames * kInCh, atomic_load(&g_in_scalar), atomic_load(&g_in_mute));
        return 0;
    case kAudioServerPlugInIOOperationWriteMix: {
        if (!s) return 0;
        const uint32_t stride = kOutCh * 3;
        uint64_t pos = (uint64_t)ci->mOutputTime.mSampleTime;
        const uint8_t *src = main;
        uint32_t done = 0;
        while (done < frames) {
            uint32_t idx = (uint32_t)((pos + done) & mask), n = frames - done;
            if (n > FTP_RING_FRAMES - idx) n = FTP_RING_FRAMES - idx;
            memcpy(s->out_ring + (size_t)idx * stride, src + (size_t)done * stride, (size_t)n * stride);
            done += n;
        }
        return 0;
    }
    case kAudioServerPlugInIOOperationReadInput: {
        const uint32_t stride = kInCh * 3;
        uint8_t *dst = main;
        if (!s) { memset(dst, 0, (size_t)frames * stride); return 0; }
        int64_t pos = (int64_t)ci->mInputTime.mSampleTime;
        int64_t newest = (int64_t)atomic_load_explicit(&s->in_newest, memory_order_acquire);
        for (uint32_t done = 0; done < frames;) {
            int64_t p = pos + done;
            uint32_t idx = (uint32_t)((uint64_t)p & mask), n = frames - done;
            if (n > FTP_RING_FRAMES - idx) n = FTP_RING_FRAMES - idx;
            // only frames in (newest - ring, newest] are valid
            if (p < 0 || p >= newest || p < newest - (int64_t)FTP_RING_FRAMES) {
                memset(dst + (size_t)done * stride, 0, (size_t)n * stride);
            } else {
                uint32_t valid = n;
                if (p + n > newest) valid = (uint32_t)(newest - p);
                memcpy(dst + (size_t)done * stride, s->in_ring + (size_t)idx * stride, (size_t)valid * stride);
                if (valid < n) memset(dst + (size_t)(done + valid) * stride, 0, (size_t)(n - valid) * stride);
            }
            done += n;
        }
        return 0;
    }
    }
    return 0;
}
