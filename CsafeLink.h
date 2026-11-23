#pragma once
#include <stdint.h>
#include <stddef.h>

// Public CSAFE standard frames only. UART and all methods owned by loop().
// A valid response/state is protocol evidence, NEVER belt-motion confirmation.
class CsafeLink {
 public:
  static constexpr uint8_t STATUS=0x80, START=0x85, STOP=0x86;
  enum Result : uint8_t { NONE, PENDING, RESPONSE, FAILED };
  uint8_t status=0, lastCommand=0;
  Result result=NONE;
  uint32_t txCount=0, rxCount=0, badFrames=0, timeouts=0;
  bool enabled=false, seen=false;

  void begin(uint32_t now) { enabled=true; nextPoll=now; }
  bool online(uint32_t now) const { return enabled && seen && now-lastRx<3000; }
  bool busy() const { return outstanding || stopLeft || startQueued; }
  bool startPending() const { return startQueued || starting; }
  bool takeStartFailure() { bool f=startFailure; startFailure=false; return f; }
  bool start(uint32_t now) {
    if (!online(now) || busy() || starting) return false;
    startQueued=true; startAt=now; result=PENDING; return true;
  }
  void stop(uint32_t now) {
    if (!enabled) return;
    startQueued=starting=false;
    // Coalesce repeated stop calls without starving the bounded retries.
    if (!stopLeft) { stopLeft=3; nextStop=now; }
    result=PENDING;
  }
  template<class Port> void tick(Port &port, uint32_t now) {
    if (!enabled) return;
    // Bound RX work so a noisy line cannot starve optical/E-stop scanning.
    for (unsigned i=0;i<64 && port.available();++i) receive(uint8_t(port.read()),now);
    if (inFrame && now-lastByte>200) { resetFrame(); ++badFrames; }
    if (outstanding && now-sentAt>=400) {
      outstanding=0; ++timeouts;
      if (lastCommand!=STATUS) result=FAILED;
    }
    if ((starting || startQueued) && now-startAt>=2500) {
      starting=startQueued=false; startFailure=true; result=FAILED;
    }
    if (outstanding || now-lastTx<100) return;
    if (stopLeft && due(now,nextStop)) {
      if (send(port,STOP,now)) { --stopLeft; nextStop=now+500; }
    } else if (!stopLeft && startQueued) {
      // Never transmit an old queued start after losing the link.
      if (!online(now)) { startQueued=false; startFailure=true; result=FAILED; }
      else if (send(port,START,now)) { startQueued=false; starting=true; }
    } else if (!stopLeft && due(now,nextPoll)) {
      if(send(port,STATUS,now)) nextPoll=now+(starting?200:1000);
    }
  }
 private:
  uint8_t outstanding=0, stopLeft=0, frame[64]{};
  size_t length=0;
  bool inFrame=false, escaped=false, startQueued=false, starting=false, startFailure=false;
  uint32_t lastRx=0,lastTx=0,sentAt=0,lastByte=0,nextPoll=0,nextStop=0,startAt=0;
  static bool due(uint32_t a,uint32_t b) { return int32_t(a-b)>=0; }
  void resetFrame() { inFrame=escaped=false; length=0; }
  template<class Port> bool send(Port &port,uint8_t command,uint32_t now) {
    // GETSTATUS appended to setters forces a reply even with acknowledgments disabled.
    const uint8_t packet[]={0xF1,command,STATUS,uint8_t(command^STATUS),0xF2};
    const uint8_t poll[]={0xF1,STATUS,STATUS,0xF2};
    const size_t n=command==STATUS?sizeof(poll):sizeof(packet);
    if (port.availableForWrite()<int(n)) return false;
    resetFrame();
    port.write(command==STATUS?poll:packet,n); // no flush/delay; bounded UART buffer write
    outstanding=command; lastCommand=command; lastTx=sentAt=now; ++txCount;
    nextPoll=now+200;
    return true;
  }
  void receive(uint8_t b,uint32_t now) {
    lastByte=now;
    if (b==0xF1) { resetFrame(); inFrame=true; return; }
    if (!inFrame) return;
    if (escaped) {
      escaped=false;
      if(b>3) { resetFrame(); ++badFrames; return; }
      b=0xF0+b;
    } else {
      if(b==0xF3) { escaped=true; return; }
      if(b==0xF0) { resetFrame(); ++badFrames; return; }
      if(b==0xF2) { finish(now); resetFrame(); return; }
    }
    if(length==sizeof(frame)) { resetFrame(); ++badFrames; return; }
    frame[length++]=b;
  }
  void finish(uint32_t now) {
    uint8_t sum=0; for(size_t i=0;i<length;++i) sum^=frame[i];
    if(length<2 || sum || (frame[0]&0x40) || (frame[0]&15)>9 || (frame[0]&15)==4) {
      ++badFrames; return;
    }
    // Reject malformed response TLVs before treating the link as live.
    for(size_t i=1;i<length-1;) {
      if(i+1>=length-1 || i+2+frame[i+1]>length-1) { ++badFrames; return; }
      i+=2+frame[i+1];
    }
    // Ignore unsolicited/late frames outside a request window.
    if(!outstanding) return;
    const uint8_t command=outstanding;
    outstanding=0; status=frame[0]; seen=true; lastRx=now; ++rxCount;
    if(command!=STATUS) result=(status&0x30)?FAILED:RESPONSE;
    if(command==START && (status&0x30)) {
      starting=false; startFailure=true; result=FAILED;
    }
    // Confirm the requested protocol state in a subsequent GETSTATUS response.
    // Still not evidence that the belt actually moved.
    if(starting && command==STATUS && !(status&0x30) && (status&15)==5) {
      starting=false; result=RESPONSE;
    }
  }
};
