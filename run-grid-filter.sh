./bin/bzrflag --world=maps/one.bzw --friendly-fire --red-port=50100 --green-port=50101 --default-true-positive=.97 --default-true-negative=.9 --occgrid-width=100 --no-report-obstacles $@ &
sleep 2
python bzagents/agent0.py localhost 50100 &
python bzagents/agent0.py localhost 50101 &
