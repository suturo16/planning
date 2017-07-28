types = {'grasp-tool': 'grasp', 'detach-tool-from-rack': 'detach', 'hold-next-to-cake': 'move-with-arm', 'place-plate-on-turtlebot': 'move-with-arm',
	 'cut-cake': 'cut', 'place-piece-of-cake-on-plate': 'move-n-flip', 'place-spatula-on-table': ('move-with-arm', 'move-gripper', 'release')}

targets = {'hold-next-to-cake': 'next2cake', 'place-plate-on-turtlebot': 'deliver', 'place-spatula-on-table': ('spatulaDropZone', 'open')}

arms = {'left': ',common:+right-arm+', 'right': ',common:+left-arm+'}


def transform_plan_to_json_string(plan):
    json_representation = "[{type: base-pose}"
  
    actions = plan.split(";")[0].split('\n')
    actions = filter(None, actions)
    
    for action in actions:
        action = action.replace('(', '').replace(')','').split()
        
        
        if (action[0] == 'grasp-tool' or action[0] == 'detach-tool-from-rack'):
            json_representation = json_representation + "," + create_json_string_with_object(types[action[0]], arms[action[1]], action[2]) #todo: reihenfolge aendern    
       
        elif (action[0] == 'hold-next-to-cake' or action[0] == 'place-plate-on-turtlebot'):
            json_representation = json_representation + "," + create_json_string_with_object_target(types[action[0]], arms[action[1]], action[2], targets[action[0]]) #todo: reihenfolge aendern    
        
        elif (action[0] == 'cut-cake'):
            json_representation = json_representation + "," + create_json_string_with_cake_knife_target(types[action[0]], arms[action[1]], action[2], action[3], action[4])
    
        elif (action[0] == 'place-piece-of-cake-on-plate'):
            json_representation = json_representation + "," + create_json_string_with_tool_target(types[action[0]], arms[action[1]], action[2], action[3]) #todo: reihenfolge aendern  

        elif (action[0] == 'place-spatula-on-table'):
             json_representation = json_representation + "," + create_json_string_with_three_types(types[action[0]][0], types[action[0]][1], types[action[0]][2], arms[action[1]], action[2], targets[action[0]][0], targets[action[0]][1])
    
        else:
            print ("Action " + str(action[0]) + " does not exist!")
    
    return json_representation + "]"


# (grasp-tool knife0 left) & (detach-tool-from-rack knife0 left)
def create_json_string_with_object(action_type, arm, object_used):
    s = "{{type: {}, arm: {}, object: {}}}"
    return s.format(action_type, arm, object_used)

# (hold-next-to-cake spatula0 right cake0) & (place-plate-on-turtlebot plate0 right)
def create_json_string_with_object_target(action_type, arm, object_used, target):
    s = "{{type: move-with-arm, arm: {}, object: {}, target: {}}}"
    return s.format(action_type, arm, object_used, target)

# (cut-cake knife0 left spatula0 right cake0 pieceofcake0)  
def create_json_string_with_cake_knife_target(action_type, arm, knife, cake, target):
    s = "{{type: {}, arm: {}, knife: {}, cake: {}, target: {}}}"
    return s.format(action_type, arm, knife, cake, target)

# (place-piece-of-cake-on-plate pieceofcake0 plate0 spatula0 right)  
def create_json_string_with_tool_target(action_type, arm, tool, target):
    s = "{{type: {}, arm: {}, tool: {}, target: {}}}"
    return s.format(action_type, arm, tool, target)

# (place-spatula-on-table spatula0 right)  
def create_json_string_with_three_types(type1, type2, type3, arm, object_used, target1, target2):
    s = "{{type: {}, arm: {}, object: {}, target: {}}}, {{type: {}, arm: {}, object: {}, target: {}}},{{type: {}, arm: {}}}"
    return s.format(type1, arm, object_used, target1, type2, arm, object_used, target2, type3, arm)
  
 
 
if __name__ == "__main__":
 
    plan = """(grasp-tool left knife)
(detach-tool-from-rack left knife)
(grasp-tool right spatula)
(hold-next-to-cake right spatula cake)
(cut-cake left knife cake spatula right pieceofcake0)
(place-piece-of-cake-on-plate right spatula plate pieceofcake0)
(place-spatula-on-table right spatula)
(grasp-tool right plate)
(place-plate-on-turtlebot right plate)
; cost = 9 (unit cost)"""

    print(transform_plan_to_json_string(plan))
    
    #print(create_json_string_with_object("a", "b", "v"))
