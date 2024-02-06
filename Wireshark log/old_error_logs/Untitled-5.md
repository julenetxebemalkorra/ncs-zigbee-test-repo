[00:01:40.147,705] <dbg> ZB_router_app: send_user_payload: 0xc4-
[00:01:40.147,705] <dbg> ZB_router_app: send_user_payload: 0x03-
[00:01:40.147,735] <dbg> ZB_router_app: send_user_payload: 0x04-
[00:01:40.147,766] <dbg> ZB_router_app: send_user_payload: 0x0b-
[00:01:40.147,796] <dbg> ZB_router_app: send_user_payload: 0x2b-
[00:01:40.147,827] <dbg> ZB_router_app: send_user_payload: 0x00-
[00:01:40.147,857] <dbg> ZB_router_app: send_user_payload: 0x1a-
[00:01:40.147,857] <dbg> ZB_router_app: send_user_payload: 0x9c-
[00:01:40.147,888] <dbg> ZB_router_app: send_user_payload: 0xd8-
[00:01:40.147,949] <dbg> ZB_router_app: send_user_payload: RET_OK - if transmission was successful scheduled;

[00:01:40.147,979] <dbg> ZB_router_app: display_counters: Output counter: 10

[00:01:41.210,266] <err> os: ***** BUS FAULT *****
[00:01:41.210,296] <err> os:   Precise data bus error
[00:01:41.210,327] <err> os:   BFAR Address: 0x20040000
[00:01:41.210,357] <err> os: r0/a1:  0x2000d6c8  r1/a2:  0x2003fffa  r2/a3:  0x2000d680
[00:01:41.210,388] <err> os: r3/a4:  0x00000006 r12/ip:  0x000019c3 r14/lr:  0x000511ab
[00:01:41.210,418] <err> os:  xpsr:  0x81000200
[00:01:41.210,449] <err> os: s[ 0]:  0x2000d680  s[ 1]:  0x00000010  s[ 2]:  0x2000d6c8  s[ 3]:  0x00000010
[00:01:41.210,479] <err> os: s[ 4]:  0x00000010  s[ 5]:  0x2000d630  s[ 6]:  0x00000001  s[ 7]:  0x0000a1e1
[00:01:41.210,510] <err> os: s[ 8]:  0x2000aab1  s[ 9]:  0x000541d8  s[10]:  0x00000000  s[11]:  0x00000000
[00:01:41.210,540] <err> os: s[12]:  0x00000000  s[13]:  0x00320010  s[14]:  0x00000000  s[15]:  0x2000d6c8
[00:01:41.210,571] <err> os: fpscr:  0xfffc6fcc
[00:01:41.210,571] <err> os: Faulting instruction address (r15/pc): 0x0005105a
[00:01:41.210,632] <err> os: >>> ZEPHYR FATAL ERROR 25: Unknown error on CPU 0
[00:01:41.210,693] <err> os: Current thread: 0x20002930 (unknown)
▒00:01:43.531,494] <err> fatal_error: Resetting system
