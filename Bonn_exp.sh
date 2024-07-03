project="Orbslam3_Yolox_withoutSet_5"
dataname="bonn"

result="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}"
result_camera="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}/Camera/"
result_keyframe="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}/KeyFrame/"

picture="/home/lzh/DATA/Exp_Results/${project}/picture/${dataname}"

datasets="/media/lzh/新加卷/Ubuntu1804_Copy/RGBD_Datesets/Boon/"
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
model=balloon
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a1=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a2=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a3=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a1" >> ${result}/final_result/mean.txt
		echo "$a2" >> ${result}/final_result/rmse.txt
		echo "$a3" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a3} != ${b1} ]]&&[ -n "$a3" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi


#-------------------------------------------------------------------------------------------
model=balloon2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a4=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a5=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a6=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a4" >> ${result}/final_result/mean.txt
		echo "$a5" >> ${result}/final_result/rmse.txt
		echo "$a6" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a6}  != ${b2} ]]&&[ -n "$a6" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi

#--------------------------------------------------------------------------------------------
model=balloon_tracking
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a7=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a8=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a9=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a7" >> ${result}/final_result/mean.txt
		echo "$a8" >> ${result}/final_result/rmse.txt
		echo "$a9" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a9} != ${b3} ]]&&[ -n "$a9" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-------------------------------------------------------------------------------------------
model=balloon_tracking2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a10=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a11=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a12=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a10" >> ${result}/final_result/mean.txt
		echo "$a11" >> ${result}/final_result/rmse.txt
		echo "$a12" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a12} != ${b4} ]]&&[ -n "$a12" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------------
model=crowd
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a13=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a14=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a15=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a13" >> ${result}/final_result/mean.txt
		echo "$a14" >> ${result}/final_result/rmse.txt
		echo "$a15" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a15}  != ${b5} ]]&&[ -n "$a9" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------
model=crowd2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a16=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a17=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a18=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a16" >> ${result}/final_result/mean.txt
		echo "$a17" >> ${result}/final_result/rmse.txt
		echo "$a18" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a18}  != ${b6} ]]&&[ -n "$a9" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------
model=crowd3
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a19=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a20=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a21=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a19" >> ${result}/final_result/mean.txt
		echo "$a20" >> ${result}/final_result/rmse.txt
		echo "$a21" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a21}  != ${b7} ]]&&[ -n "$a9" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi

		#---------------------------------------------------------------------------------------------
model=kidnapping_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a22=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a23=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a24=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a22" >> ${result}/final_result/mean.txt
		echo "$a23" >> ${result}/final_result/rmse.txt
		echo "$a24" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a24}  != ${b8} ]]&&[ -n "$a24" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
				#---------------------------------------------------------------------------------------------
model=kidnapping_box2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a25=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a26=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a27=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a25" >> ${result}/final_result/mean.txt
		echo "$a26" >> ${result}/final_result/rmse.txt
		echo "$a27" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a27} != ${b9} ]]&&[ -n "$a27" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#------------------------------------------------------------------------------------------------
model=moving_nonobstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a28=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a29=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a30=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a28" >> ${result}/final_result/mean.txt
		echo "$a29" >> ${result}/final_result/rmse.txt
		echo "$a30" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a30}  != ${b10} ]]&&[ -n "$a30" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------------
model=moving_nonobstructing_box2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a31=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a32=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a33=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a31" >> ${result}/final_result/mean.txt
		echo "$a32" >> ${result}/final_result/rmse.txt
		echo "$a33" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a33}  != ${b11} ]]&&[ -n "$a33" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-------------------------------------------------------------------------------------------
model=moving_obstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a34=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a35=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a36=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a34" >> ${result}/final_result/mean.txt
		echo "$a35" >> ${result}/final_result/rmse.txt
		echo "$a36" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a36}  != ${b12} ]]&&[ -n "$a36" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------
model=moving_obstructing_box2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a37=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a38=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a39=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a37" >> ${result}/final_result/mean.txt
		echo "$a38" >> ${result}/final_result/rmse.txt
		echo "$a39" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a39} != ${b13} ]]&&[ -n "$a39" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#--------------------------------------------------------------------------------------------
model=person_tracking
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a40=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a41=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a42=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a40" >> ${result}/final_result/mean.txt
		echo "$a41" >> ${result}/final_result/rmse.txt
		echo "$a42" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a42}  != ${b14} ]]&&[ -n "$a42" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#--------------------------------------------------------------------------------
model=person_tracking2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a43=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a44=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a45=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a43" >> ${result}/final_result/mean.txt
		echo "$a44" >> ${result}/final_result/rmse.txt
		echo "$a45" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a45}  != ${b15} ]]&&[ -n "$a45" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#--------------------------------------------------------------------------------------------
model=placing_nonobstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a46=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a47=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a48=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a46" >> ${result}/final_result/mean.txt
		echo "$a47" >> ${result}/final_result/rmse.txt
		echo "$a48" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a48}  != ${b16} ]]&&[ -n "$a48" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#--------------------------------------------------------------------------------------------
model=placing_nonobstructing_box2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a49=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a50=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a51=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a49" >> ${result}/final_result/mean.txt
		echo "$a50" >> ${result}/final_result/rmse.txt
		echo "$a51" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a51}  != ${b17} ]]&&[ -n "$a51" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------------
model=placing_nonobstructing_box3
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a52=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a53=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a54=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a52" >> ${result}/final_result/mean.txt
		echo "$a53" >> ${result}/final_result/rmse.txt
		echo "$a54" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a54} != ${b18} ]]&&[ -n "$a54" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------
model=placing_obstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a55=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a56=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a57=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a55" >> ${result}/final_result/mean.txt
		echo "$a56" >> ${result}/final_result/rmse.txt
		echo "$a57" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a57}  != ${b19} ]]&&[ -n "$a57" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------
model=removing_nonobstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a58=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a59=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a60=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a58" >> ${result}/final_result/mean.txt
		echo "$a59" >> ${result}/final_result/rmse.txt
		echo "$a60" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a60}  != ${b20} ]]&&[ -n "$a60" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-----------------------------------------------------------------------------------------------
model=removing_nonobstructing_box2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a61=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a62=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a63=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a61" >> ${result}/final_result/mean.txt
		echo "$a62" >> ${result}/final_result/rmse.txt
		echo "$a63" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a63}  != ${b21} ]]&&[ -n "$a63" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#----------------------------------------------------------------------------------------------
model=removing_obstructing_box
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a64=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a65=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a66=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a64" >> ${result}/final_result/mean.txt
		echo "$a65" >> ${result}/final_result/rmse.txt
		echo "$a66" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a66}  != ${b22} ]]&&[ -n "$a66" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
		#----------------------------------------------------------------------------------------------
model=static
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a67=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a68=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a69=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a67" >> ${result}/final_result/mean.txt
		echo "$a68" >> ${result}/final_result/rmse.txt
		echo "$a69" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a69}  != ${b23} ]]&&[ -n "$a69" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-----------------------------------------------------------------------------------------------
model=static_close_far
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a70=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a71=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a72=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a70" >> ${result}/final_result/mean.txt
		echo "$a71" >> ${result}/final_result/rmse.txt
		echo "$a72" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a72}  != ${b24} ]]&&[ -n "$a72" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#-----------------------------------------------------------------------------------------------
model=synchronous
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a73=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a74=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a75=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a73" >> ${result}/final_result/mean.txt
		echo "$a74" >> ${result}/final_result/rmse.txt
		echo "$a75" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a75} != ${b25} ]]&&[ -n "$a75" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------
model=synchronous2
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/Bonn_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a76=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a77=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a78=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a76" >> ${result}/final_result/mean.txt
		echo "$a77" >> ${result}/final_result/rmse.txt
		echo "$a78" >> ${result}/final_result/std.txt
		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a78}  != ${b26} ]]&&[ -n "$a78" ];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
break
#---------------------------------------------------------------------------------------
done