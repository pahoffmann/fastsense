#!/bin/bash
ssh root@board 'date --set $(date "+%Y-%m-%dT%H:%M:%S")'
