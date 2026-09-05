#!/usr/bin/env python3
"""Timeline of byte changes per CAN ID from a candump -l capture: prints every
moment any byte of a frame ID changes value (skipping pure counters), so a
sequence of rider actions maps onto the bytes that reacted."""
import sys, collections
lines=[l.split() for l in open(sys.argv[1]) if l.startswith('(')]
t0=float(lines[0][0].strip('()'))
if len(sys.argv)>2: t0=float(sys.argv[2])   # optional absolute start (epoch)
last={}; changes=collections.defaultdict(int)
events=[]
for l in lines:
    t=float(l[0].strip('()')); cid,data=l[2].split('#')
    if cid=='7E0': continue
    prev=last.get(cid)
    if prev is not None and prev!=data:
        for b in range(0,min(len(prev),len(data)),2):
            if prev[b:b+2]!=data[b:b+2]:
                changes[(cid,b//2)]+=1
                events.append((t,cid,b//2,prev[b:b+2],data[b:b+2]))
    last[cid]=data
counters={k for k,v in changes.items() if v>len(lines)/40}   # changes nearly every frame = counter
print(f"{len(lines)} frames over {float(lines[-1][0].strip('()'))-t0:.1f}s; skipping counters: {sorted(counters)}")
lastprint={}
for t,cid,b,a,c in events:
    if (cid,b) in counters: continue
    key=(cid,b); 
    print(f"t+{t-t0:7.2f}s  {cid:>9} byte{b}: {a} -> {c}")
