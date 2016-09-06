#!/usr/bin/env bash
# sleep 5;
# source ~/.bashrc
# rossethrp2016
rosservice call /transformable_server_sample/request_marker_operate "operate: {type: 0, action: 0, frame_id: 'kinfu_origin', name: 'dummy_box', description: '', mesh_resource: '', mesh_use_embedded_materials: false}"
rosservice call /transformable_server_sample/set_color "color: {r: 0.0, g: 1.0, b: 1.0, a: 0.5}"
