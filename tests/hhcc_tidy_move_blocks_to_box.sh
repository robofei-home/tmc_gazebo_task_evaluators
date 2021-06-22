#!/bin/sh
# Copyright (c) 2019 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

rosservice call /gazebo/delete_model '{model_name: block0}'
rosservice call /gazebo/delete_model '{model_name: block1}'
rosservice call /gazebo/delete_model '{model_name: block2}'
rosservice call /gazebo/delete_model '{model_name: block3}'
rosservice call /gazebo/delete_model '{model_name: block4}'
rosservice call /gazebo/delete_model '{model_name: block5}'
rosservice call /gazebo/delete_model '{model_name: block6}'
rosservice call /gazebo/delete_model '{model_name: block7}'
rosservice call /gazebo/delete_model '{model_name: block8}'
rosservice call /gazebo/delete_model '{model_name: block9}'
rosservice call /gazebo/delete_model '{model_name: block10}'
rosservice call /gazebo/delete_model '{model_name: block11}'

rosrun hsrb_hhcc_gazebo_worlds spawn_blocks  --number 12 --seed 1 -x -2 -y 4 -z 1.0 --distribution 0.1
