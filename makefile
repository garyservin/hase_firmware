# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT         := hase_firmware_ROS
DEVICES         := LPC1768 PAC_F401RB
GCC4MBED_DIR    := /home/gary/devel/mbed/gcc4mbed
USER_LIBS       := $(ROS_LIB_DIR)
INCDIRS         := libraries
NO_FLOAT_SCANF  := 1

ifdef GCC4MBED_RESET
reset:
	@echo "Resetting the target"
	$(GCC4MBED_RESET)
endif

ifdef GCC4MBED_RESET_BBB
reset-bbb:
	@echo "Resetting the Mbed remotely"
	$(GCC4MBED_RESET_BBB)
endif

ifdef GCC4MBED_DEPLOY_BBB
DEPLOY_COMMAND = $(subst PROJECT,$(DEVICES)/$(PROJECT),$(GCC4MBED_DEPLOY_BBB))
deploy-bbb:
	@echo "Deploying to target remotely and resetting"
	$(DEPLOY_COMMAND)
	@echo "Resetting the Mbed remotely"
	$(GCC4MBED_RESET_BBB)
endif

include $(GCC4MBED_DIR)/build/gcc4mbed.mk
