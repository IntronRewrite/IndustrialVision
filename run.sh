cd /home/lw/IndustrialVision/build
rm -rf ./*
cmake ..
make
./RegistrationRANSAC  /home/lw/IndustrialVision/src_20w.ply /home/lw/IndustrialVision/ref_20w.ply  --method=feature_matching --voxel_size=50  --distance_multiplier=1.5 --max_iterations=1000000 --confidence=0.999 --mutual_filter
./RegistrationICP  /home/lw/IndustrialVision/src_20w.ply /home/lw/IndustrialVision/ref_20w.ply  --visualize