# A consensus sync point (an engine-management decision a super-majority of
# peers agreed upon). `payload` is empty until upstream tashi-vertex exposes
# sync-point accessors (see README open gap, design §9.2).

builtin_interfaces/Time observed_at
uint8[]                 payload
