#!/bin/sh
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

# warn-unclean: print a warning if the working tree and index are not clean
#
# For utmost strictness, set check_untracked=yes and check_ignored=yes.
# When both are 'yes', verify that working tree and index are identical to HEAD.
# When only check_untracked is yes, extra ignored files are allowed.
# When neither is yes, extra untracked files and ignored files are allowed.

check_untracked=yes
check_ignored=no

git_string=''

# Compare HEAD to index and/or working tree versions of tracked files
git diff-index --quiet HEAD
case $? in
    0)
        if test "$check_untracked" != yes; then
            clean=yes
        else
            # Still need to check for untracked files
            exclude=--exclude-standard
            if test "$check_ignored" = yes; then
                exclude=''
            fi

            (
                # Move to top level of working tree
                if up="$(git rev-parse --show-cdup)"; then
                    test -n "$up" && cd "$up"
                else
                    echo 'error running "git rev-parse --show-cdup"'
                    exit 129
                fi

                # Check for untracked files
                git ls-files --others $exclude --error-unmatch . >/dev/null 2>&1
                case $? in
                    0)  # some untracked/ignored file is present
                        exit 1
                    ;;
                    1)  # no untracked files
                        exit 0
                    ;;
                    *)
                        echo 'error running "git diff-index"!'
                        exit 129
                    ;;
                esac

            )
            case $? in
                0) clean=yes ;;
                1) clean=no ;;
                *) exit $? ;;
            esac
        fi
    ;;
    1)
        clean=no
    ;;
    *)
        echo 'error running "git diff-index"!'
        exit 129
    ;;
esac

if c="$(git rev-parse --verify HEAD)"; then
    git_string="$c" # ' '"$git_string"
else
    echo 'error running "git rev-parse --verify"!'
fi

if test "$clean" != yes; then
  git_string="$git_string"' (unclean)'
fi

echo "$git_string"
exit 0

