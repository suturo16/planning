rosservice call /json_prolog/simple_query "mode: 0 
id: '0' 
query: 'dummy_perception(cylinder)'"
rosservice call /json_prolog/next_solution "id: '0'"
rosservice call /json_prolog/simple_query "mode: 0 
id: '1' 
query: 'get_object_infos(knowrob:cylinder, Frame, Height, Width, Depth)'"
rosservice call /json_prolog/next_solution "id: '1'"