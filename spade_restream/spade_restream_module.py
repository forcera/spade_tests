# Copyright (c) 2024 FORCERA, LDA
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

import rtsp_utils

output_url_data = {
    "ip": "0.0.0.0",  # nosec B104
    "port": "31415",
    "name": "/spade_stream",
    "stream_width": 1280,
    "stream_height": 720,
    "stream_fps": 30,
    "stream_pxl_format": "yuv420p"
}

restream_obj = rtsp_utils.restreaming(output_url_data)
restream_obj.run_restream()
