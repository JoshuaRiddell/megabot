#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

rsync -az $DIR/.. megabotpi:~/megabot --exclude={'devel','build'}
