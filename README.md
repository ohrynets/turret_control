= Turret Robot with Llm brain

== Turret Robot Gazeebo emulation inside a world with two copters

Run multimodal *llava* node to process images and text
```bash
export LLAMA_CUDA="on"
ros2 launch llama_bringup llava.launch.py
```
Run llama_node to receive message from */slack_in* topic, call llm model node, and send the result to */slack_out* topic
```bash
ros2 run turret_control llama_node --ros-args --params-file src/turret_control/config/slack_llm.yaml
``` 
To run *slack_node*, which listen for slack message and redirect them to /slack_in topic.
Export Slack Token:
```bash
export SLACK_TOKEN=xoxb-YOUR-TOKEN-THERE
```

Run *slack_node*:
```bash
ros2 run turret_control slack_node --ros-args --params-file src/turret_control/config/slack_llm.yaml -p slack_token:=$SLACK_TOKEN
```




