# UBC Open Robotics Integration Docs

This is a summary of the current components for the Robocup@Home Robot and some pseudocode for the planned integration.

## Robocup@Home tasks (Pseudocode)

### Carry My Luggage

#### Setup

```python
nav.map_room('frontier exploration')
nav.name_locations(['initial', 'front door'])
```

#### Initialization

```python
vis.launch(vis.objection_detection.pub(['person', 'bag']))
nav.launch(nav.slam.srv(room_map))
arm.launch(arm.controller.srv)
nlp.launch(nlp.speech_to_text.pub) # from microphone
```

#### Main

```python
class Robot():
	speech_buffer = CircularBuffer()
    person_location = Location()
    bag_location = Location()
    
    def __init__(self):
        speech_sub = subscribe(nlp.speech_to_text.topic, callback=self.save_speech)
        person_sub = subscribe(vis.objection_detection.topic, callback=self.update_obj_loc)
        self.map = nav.load_map(nav.named_locations)
        
    def save_speech(self, msg):
        self.speech_buffer.append(msg)
        
    def update_obj_loc(self, msg):
        self.person_location = msg.person_location # Do we need to handle multiple people?
        self.bag_location = msg.bag_location
        
    def main(self):
        nav.self_localize()
        ros.call(nav.named_location.srv, 'GOTO front door')
        ros.call(nav.slam.goto(loc=self.person_location))
        nlp.story(['welcome', 'Let me take your bag'])
		ros.call(arm.gripper.goto(loc=self.bag_location)) # Will we need handle? Do we need to do handover
        while not (nlp.analyse(self.speech_buffer, find='stop signal from person')):
            ros.call(nav.slam.goto(loc=self.person_location)) 
            # what if we miss person? (last update is too old) Recovery action.
            if nav.slam.check_unovercomeable_object():
                recovery()
        image = ros.wait_for_message(camera_topic)
        car_location = vis.objection_detection.model(['car'])(image)
        ros.call(nav.slam.goto(loc=car_location))
        
        
```





## Components Quick Reference

### Navigation

[UBC-OpenRobotics/navigation: Navigation for turtlebot2 (github.com)](https://github.com/UBC-OpenRobotics/navigation)

#### Summary (Pseudonyms)

**Scripts** (can rosrun):

* map_server
  * Is run to build a map file. Automatically builds from consecutive lidar scans
  * subscribes to: lidar
  * outputs to: specified map file
* frontier_exploration
  * subscribes to lidar, map file
  * publishes to , move_base (navigation controller)

**Servers:**

* named_location
  * saves or goes to a location with a string as tag
* slam
  * goes to specified coordinate

#### Building a Map

Current map building method is by using the gmapping ROS package along with a mode of movement around the environment.

1. **Bringup Robot (must include move_base)**

2. **Launch the mapping server:** 

```shell
rosrun map_server map_saver -f ~/map
```

3. **Move around environment using:** 
   1. Keyboard: ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py``` 
   2. Frontier Exploration: ```roslaunch navigation frontier_exploration.launch```

#### Navigation Modes

##### Named Location

Implemented as server. Can call GOTO or NAME

GOTO goes to saved location (name) in map if exists

NAME tags current robot location with name (string)

Examples: 

```shell
rostopic pub /navigate_named_location navigation/NamedLocation '{action: NAME, name: kitchen}'
rostopic pub /navigate_named_location navigation/NamedLocation '{action: GOTO, name: 'front door'}'
```

### Markdown

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [Basic writing and formatting syntax](https://docs.github.com/en/github/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).
