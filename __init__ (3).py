# GBNF grammar for planner output (llama.cpp). Used when
# config.PLANNER_CONSTRAIN_MODE == "gbnf". Must stay in sync with
# actions/goal.py ACTIONS and config.ZONES.
#
# TODO: zone ids and priority range are hard-coded here; regenerate if you
# change config.ZONES. The json_schema mode (default) does not have this
# problem because llama.cpp derives the grammar from tools.RESPONSE_SCHEMA.
# TODO: tune if the 230M model still emits junk with this grammar (e.g. add
# whitespace rules or forbid the empty list).

root        ::= "{\"tool_calls\":[" calls? "]}"
calls       ::= call ("," call)?
call        ::= nav | explore | approach | conceal | hold | scan

nav         ::= "{\"name\":\"navigate_to_zone\",\"arguments\":{\"zone_id\":\"" zone "\"},\"priority\":" prio "}"
explore     ::= "{\"name\":\"explore_until_found\",\"arguments\":{\"ttl_seconds\":" num "},\"priority\":" prio "}"
approach    ::= "{\"name\":\"approach_target\",\"arguments\":{\"standoff_meters\":" num "},\"priority\":" prio "}"
conceal     ::= "{\"name\":\"reposition_for_concealment\",\"arguments\":{},\"priority\":" prio "}"
hold        ::= "{\"name\":\"hold_position\",\"arguments\":{" holdargs? "},\"priority\":" prio "}"
holdargs    ::= "\"seconds\":" num
scan        ::= "{\"name\":\"scan_with_camera\",\"arguments\":{\"pan_deg\":" snum ("," "\"tilt_deg\":" snum)? "},\"priority\":" prio "}"

zone        ::= "zone_a" | "zone_b" | "zone_c" | "zone_d"
prio        ::= [0-9] | [1-7] [0-9]
num         ::= [0-9]+ ("." [0-9]+)?
snum        ::= "-"? num
