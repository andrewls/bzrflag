./bin/bzrflag --world=maps/grid.bzw --friendly-fire --red-port=50100 --green-port=50101 --default-true-positive=.7 --default-true-negative=.65 --occgrid-width=100 --no-report-obstacles --red-tanks=10 $@ &
sleep 2
python bzagents/grid-agent.py localhost 50100 &
