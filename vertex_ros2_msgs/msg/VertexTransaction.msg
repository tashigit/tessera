# A single opaque transaction submitted to (or replayed out of) the Vertex
# consensus engine.
#
# Vertex treats `payload` as opaque bytes; it neither inspects nor transmits
# `created_at` or `tag`. Those two fields are ROS-side metadata only:
#   - On /vertex/tx (inbound) they are advisory and ignored by v0.1.
#   - On /vertex/event (outbound) they are not recoverable per-transaction and
#     are left at their defaults (zero time, empty tag). See README "Semantics".

builtin_interfaces/Time created_at
uint8[]                 payload
string                  tag
