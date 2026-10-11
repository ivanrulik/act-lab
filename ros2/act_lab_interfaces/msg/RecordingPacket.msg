# One ordered, acknowledged stream per producer. Version 1.
std_msgs/Header header
uint32 schema_version
string episode_id
uint64 sequence
string kind
act_lab_interfaces/RecordedSample sample
string metadata_json
string outcome
string reason
