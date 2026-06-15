# A consensus sync point (an engine-management decision a super-majority of
# peers agreed upon). Carries the application-actionable fields: the epoch this
# sync point begins (the cue to checkpoint application state), whether the node
# just synced into the session, and whether the session ended (design §9.2).

builtin_interfaces/Time observed_at
uint64                  epoch_index
bool                    just_synced
bool                    session_ended
