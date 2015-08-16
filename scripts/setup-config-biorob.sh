
mkdir e
mkdir o
mkdir txt
mkdir pbs

module load application/python/2.7.3
module load python/2.7.2
module load python

cd results/robot-experiments/biorob/
python configprob.py
cd td/
python configalg_numerical.py
cd ../tdr
python configalg_numerical.py
cd ../totd
python configalg_numerical.py
cd ../autotd
python configalg_numerical.py
echo "setup configs"
