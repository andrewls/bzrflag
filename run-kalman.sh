./bin/bzrflag --world=maps/empty.bzw --friendly-fire --red-port=50100 --blue-port=50101 --red-tanks=1 --blue-tanks=1 $@ &
sleep 2
python bzagents/kalman-agent.py localhost 50100 &
# python bzagents/constant-clay-pigeon.py localhost 50101 &
python bzagents/variable-speed-agent.py localhost 50101 &
