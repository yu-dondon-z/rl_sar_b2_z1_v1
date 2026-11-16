#!/usr/bin/env python3
"""VD13 mount75 model_6199 + rendered RGB/RGB-D + RHANet; simulation only.

No truth fallback, fake frame freshness, actor export or robot I/O.
"""
from deploy_vd12_vision import main

if __name__ == '__main__':
    raise SystemExit(main(version=13))
