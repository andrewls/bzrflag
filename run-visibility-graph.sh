./bin/bzrflag --world=maps/one.bzw  --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &

sleep 2
python bzagents/path-agent.py localhost 50100 &
