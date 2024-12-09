# Copyright (c) 2024 FORCERA, LDA
# 
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0

# Use the official Ubuntu as the base image
FROM ubuntu:22.04

# Set the working directory
WORKDIR /usr/src/app

# Install dependencies and Redis
RUN apt-get update && \
    apt-get install -y redis-server python3 python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Expose the default Redis port
EXPOSE 6379

# Copy entrypoint script
COPY start.sh .
RUN chmod +x start.sh

# Set the entrypoint to use tmux to start Redis
ENTRYPOINT ["./start.sh"]
