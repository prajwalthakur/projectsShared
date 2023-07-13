#! /usr/bin/env bash
SESSION = "vscode 'pwd | md5sum | cut -b -3' "
echo $SESSION
tmux attach-session -d -t $SESSION || tmux new-session -s $SESSION