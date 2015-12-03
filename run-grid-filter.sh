./bin/bzrflag --world=maps/maze1.bzw --friendly-fire --red-port=50100 --green-port=50101 --default-true-positive=.97 --default-true-negative=.9 --occgrid-width=100 --no-report-obstacles --red-tanks=10 $@ &
sleep 2
python bzagents/grid-agent.py localhost 50100 &
