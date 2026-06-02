# A Vertex event that has reached consensus. Published on /vertex/event in the
# exact order Vertex finalizes events; transactions within an event are ordered
# by Vertex's index 0..n.

builtin_interfaces/Time consensus_at        # event.consensus_at()
builtin_interfaces/Time created_at          # event.created_at()
uint8[32]               hash                # event.hash()
uint8[]                 creator_pub_der     # DER-encoded KeyPublic of the creator
uint8[]                 whitened_signature  # event.whitened_signature()
VertexTransaction[]     transactions        # in Vertex's order
