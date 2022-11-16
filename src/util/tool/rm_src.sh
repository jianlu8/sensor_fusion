#!/bin/bash

find . -name '*.c' -type f -print -exec rm {} \;
find . -name '*.cpp' -type f -print -exec rm {} \;
find . -name '*.cc' -type f -print -exec rm {} \;
find . -name '*.h' -type f -print -exec rm {} \;
find . -name '*.hpp' -type f -print -exec rm {} \;
find . -name '*.hh' -type f -print -exec rm {} \;
find . -name .git -type d -print -exec rm -rf {} \;
