# ./bin/bzrflag --world=maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
./bin/bzrflag --world=maps/pacman.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
sleep 2
python bzagents/pf-agent.py localhost 50100 &
python bzagents/really-dumb-agent.py localhost 50101 &
python bzagents/really-dumb-agent.py localhost 50102 &
python bzagents/really-dumb-agent.py localhost 50103 &
