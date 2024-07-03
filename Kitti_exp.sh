project="Orbslam3_Yolox_withoutSet_5"
dataname="kitti"

result="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}"
result_camera="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}/Camera/"


picture="/home/lzh/DATA/Exp_Results/${project}/picture/${dataname}"

datasets="/media/lzh/新加卷/Ubuntu1804_Copy/kitti_dataset_stereo_rgb/sequences/"
camera_exp="/home/lzh/VINS/SPL_SLAM"
picture="/home/lzh/DATA/Exp_Results/${project}/picture/${dataname}"

b1=1
b2=1
b3=1
b4=1
b5=1
b6=1
b7=1
b8=1
b9=1
b10=1
b11=1
b12=1
b13=1
b14=1
b15=1
b16=1
b17=1
b18=1
b19=1
b20=1
b21=1
b22=1
b23=1
b24=1
b25=1
b26=1


while true
do
model=00
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI00-02.yaml ${datasets}${model}
		a1=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a2=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a3=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a1" >> ${result}/final_result/mean.txt
		echo "$a2" >> ${result}/final_result/rmse.txt
		echo "$a3" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a3} < ${b1} ]]&&[ -n "$a3" ];then
			b1=${a1}
		
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi


#-------------------------------------------------------------------------------------------
model=01
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI00-02.yaml ${datasets}${model} 
		a4=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a5=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a6=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a4" >> ${result}/final_result/mean.txt
		echo "$a5" >> ${result}/final_result/rmse.txt
		echo "$a6" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		
	# 检查是否为空
		if [[ ${a4} != 0  ]]&&[ -n "$a6" ];then
			
	
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi

#--------------------------------------------------------------------------------------------
model=02
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI00-02.yaml ${datasets}${model}  
		a7=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a8=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a9=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a7" >> ${result}/final_result/mean.txt
		echo "$a8" >> ${result}/final_result/rmse.txt
		echo "$a9" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a7} != 0 ]]&&[ -n "$a9" ];then
			b1=${a1}
	
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-------------------------------------------------------------------------------------------
model=03
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI03.yaml ${datasets}${model}  
		a10=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a11=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a12=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a10" >> ${result}/final_result/mean.txt
		echo "$a11" >> ${result}/final_result/rmse.txt
		echo "$a12" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a12} < ${b4} ]]&&[ -n "$a12" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------------
model=04
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a13=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a14=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a15=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a13" >> ${result}/final_result/mean.txt
		echo "$a14" >> ${result}/final_result/rmse.txt
		echo "$a15" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a15} < ${b5} ]]&&[ -n "$a9" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------
model=05
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a16=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a17=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a18=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a16" >> ${result}/final_result/mean.txt
		echo "$a17" >> ${result}/final_result/rmse.txt
		echo "$a18" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a18} < ${b6} ]]&&[ -n "$a9" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------
model=06
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model} 
		a19=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a20=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a21=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a19" >> ${result}/final_result/mean.txt
		echo "$a20" >> ${result}/final_result/rmse.txt
		echo "$a21" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a21} < ${b7} ]]&&[ -n "$a9" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi

		#---------------------------------------------------------------------------------------------
model=07
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a22=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a23=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a24=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a22" >> ${result}/final_result/mean.txt
		echo "$a23" >> ${result}/final_result/rmse.txt
		echo "$a24" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a24} < ${b8} ]]&&[ -n "$a24" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
				#---------------------------------------------------------------------------------------------
model=08
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a25=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a26=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a27=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a25" >> ${result}/final_result/mean.txt
		echo "$a26" >> ${result}/final_result/rmse.txt
		echo "$a27" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a27} < ${b9} ]]&&[ -n "$a27" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#------------------------------------------------------------------------------------------------
model=09
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a28=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a29=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a30=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a28" >> ${result}/final_result/mean.txt
		echo "$a29" >> ${result}/final_result/rmse.txt
		echo "$a30" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a30} < ${b10} ]]&&[ -n "$a30" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------------
model=10
./Examples/Stereo-Kitti/stereo_line_kitti Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/Stereo-Kitti/KITTI04-12.yaml ${datasets}${model}  
		a31=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a32=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a33=`evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a31" >> ${result}/final_result/mean.txt
		echo "$a32" >> ${result}/final_result/rmse.txt
		echo "$a33" >> ${result}/final_result/std.txt
		evo_ape kitti ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png


		
		
		# 检查是否为空
		if [[ ${a33} < ${b11} ]]&&[ -n "$a33" ];then
			b1=${a1}

			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
break
#---------------------------------------------------------------------------------------
done