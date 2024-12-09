# Copyright (c) 2024 FORCERA, LDA
# 
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0

#!/bin/bash

python3 ./framework/spade_drone_module.py &

# Wait for all background processes to finish (this keeps the container alive)
wait