--This file shows how to setting up luban system within an new ubuntu--

#1. setup algorithms service with self-start
    chmod +x algorithms.sh
    sudo cp algorithms.sh /opt/knowin
    sudo cp algorithms.service /etc/systemd/system
    sudo systemctl daemon-reload
    sudo systemctl enable algorithms.service

#2 alias command for algorithms
    alias alg_sst='sudo systemctl start algorithms'
    alias alg_ssp='sudo systemctl stop algorithms'
    alias alg_sss='sudo systemctl status algorithms'
    alias alg_log='tail -f /opt/knowin/logs/algorithm.log'

#3. reboot you ubuntu
