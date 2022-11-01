TIMEOUT(50000);

var failed = false;
var done = 0;

while(done < sim.getMotes().length) {
    YIELD();

    log.log(time + " " + "node-" + id + " "+ msg + "\n");

    if(msg.contains("=check-me=") == false) {
        continue;
    }

    if(msg.contains("FAILED")) {
        failed = true;
    }

    if(msg.contains("DONE")) {
        done++;
    }
}
if(failed) {
    log.testFailed();
}
log.testOK();
