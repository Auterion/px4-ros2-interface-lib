#ifndef PX4_MSGS_ALL_MESSAGES_HPP
#define PX4_MSGS_ALL_MESSAGES_HPP

// Auto-generated header including all PX4 message types
@{
import re

def remove_suffix(text: str, suffix: str) -> str:
    if text.endswith(suffix):
        return text[:-len(suffix)]
    return text

def camel_to_snake(name):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()

for msg in message_names:
    print("#include <px4_msgs/msg/" + camel_to_snake(remove_suffix(msg, ".msg")) + ".hpp>")
}@

struct TopicTypeSupport
{
  const char * topic_type_name;
  const rosidl_message_type_support_t * ts_handle;
};

static TopicTypeSupport all_px4_ros2_messages[] = {
@{
for msg_file in message_names:
    msg = remove_suffix(msg_file, ".msg")
    print("  {\"px4_msgs/msg/" + msg + "\",")
    print("    rosidl_typesupport_cpp::get_message_type_support_handle<px4_msgs::msg::" + msg + ">()},")
}@
};

const TopicTypeSupport* find_type_support(const std::string& type_name)
{
  for (const auto& entry : all_px4_ros2_messages) {
    if (type_name == entry.topic_type_name) {
      return &entry;
    }
  }
  return nullptr; // Not found
}


#endif // PX4_MSGS_ALL_MESSAGES_HPP
