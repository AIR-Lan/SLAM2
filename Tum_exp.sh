project="Orbslam3_Yolox_withoutSet_5"
dataname="tum"

result="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}"
result_camera="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}/Camera/"
result_keyframe="/home/lzh/DATA/Exp_Results/${project}/data/${dataname}/KeyFrame/"

picture="/home/lzh/DATA/Exp_Results/${project}/picture/${dataname}"

datasets="/media/lzh/新加卷/Ubuntu1804_Copy/RGBD_Datesets/TUM/"
camera_exp="/home/lzh/VINS/SPL_SLAM"


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
# model=desk_with_person
# ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
# 		a1=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a2=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a3=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a1" >> ${result}/final_result/mean.txt
# 		echo "$a2" >> ${result}/final_result/rmse.txt
# 		echo "$a3" >> ${result}/final_result/std.txt

# 		a4=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a5=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a6=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a4" >> ${result}/final_result/rpe/mean.txt
# 		echo "$a5" >> ${result}/final_result/rpe/rmse.txt
# 		echo "$a6" >> ${result}/final_result/rpe/std.txt

# 		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

# 		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
# 		# 检查是否为空
# 		if [[ ${a3} < ${b1} ]]&&[ -n "$a3" ];then
# 			b1=${a1}
# 			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
# 			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
# 			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
# 			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
# 		fi


#-------------------------------------------------------------------------------------------


#--------------------------------------------------------------------------------------------
# model=sitting_rpy
# ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
# 		a13=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a14=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a15=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a13" >> ${result}/final_result/mean.txt
# 		echo "$a14" >> ${result}/final_result/rmse.txt
# 		echo "$a15" >> ${result}/final_result/std.txt

# 		a16=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a17=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a18=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a16" >> ${result}/final_result/rpe/mean.txt
# 		echo "$a17" >> ${result}/final_result/rpe/rmse.txt
# 		echo "$a18" >> ${result}/final_result/rpe/std.txt

# 		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

# 		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
# 		# 检查是否为空
# 		if [[ ${a15} < ${b3} ]]&&[ -n "$a9" ];then
# 			b1=${a1}
# 			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
# 			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
# 			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
# 			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
# 		fi
#-------------------------------------------------------------------------------------------
# model=sitting_static
# ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
# 		a19=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a20=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a21=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a19" >> ${result}/final_result/mean.txt
# 		echo "$a20" >> ${result}/final_result/rmse.txt
# 		echo "$a21" >> ${result}/final_result/std.txt

# 		a22=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
# 		a23=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
# 		a24=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
# 		echo "$a22" >> ${result}/final_result/rpe/mean.txt
# 		echo "$a23" >> ${result}/final_result/rpe/rmse.txt
# 		echo "$a24" >> ${result}/final_result/rpe/std.txt

# 		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

# 		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
# 		# 检查是否为空
# 		if [[ ${a21} < ${b4} ]]&&[ -n "$a12" ];then
# 			b1=${a1}
# 			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
# 			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
# 			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
# 			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
# 		fi
#---------------------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
model=walking_halfsphere
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a31=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a32=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a33=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a31" >> ${result}/final_result/mean.txt
		echo "$a32" >> ${result}/final_result/rmse.txt
		echo "$a33" >> ${result}/final_result/std.txt

		a34=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a35=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a36=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a34" >> ${result}/final_result/rpe/mean.txt
		echo "$a35" >> ${result}/final_result/rpe/rmse.txt
		echo "$a36" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a33} != ${b6} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
#---------------------------------------------------------------------------------------------
model=walking_rpy
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a37=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a38=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a39=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a37" >> ${result}/final_result/mean.txt
		echo "$a38" >> ${result}/final_result/rmse.txt
		echo "$a39" >> ${result}/final_result/std.txt

		a40=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a41=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a42=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a40" >> ${result}/final_result/rpe/mean.txt
		echo "$a41" >> ${result}/final_result/rpe/rmse.txt
		echo "$a42" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a39} != ${b7} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi

		#---------------------------------------------------------------------------------------------
model=walking_static
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a43=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a44=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a45=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a43" >> ${result}/final_result/mean.txt
		echo "$a44" >> ${result}/final_result/rmse.txt
		echo "$a45" >> ${result}/final_result/std.txt

		a46=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a47=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a48=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a46" >> ${result}/final_result/rpe/mean.txt
		echo "$a47" >> ${result}/final_result/rpe/rmse.txt
		echo "$a48" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a45} != ${b8} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
				#---------------------------------------------------------------------------------------------
model=walking_xyz
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a49=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a50=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a51=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a49" >> ${result}/final_result/mean.txt
		echo "$a50" >> ${result}/final_result/rmse.txt
		echo "$a51" >> ${result}/final_result/std.txt

		a52=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a53=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a54=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a52" >> ${result}/final_result/rpe/mean.txt
		echo "$a53" >> ${result}/final_result/rpe/rmse.txt
		echo "$a54" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a51} != ${b9} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
model=sitting_halfsphere
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a7=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a8=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a9=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a7" >> ${result}/final_result/mean.txt
		echo "$a8" >> ${result}/final_result/rmse.txt
		echo "$a9" >> ${result}/final_result/std.txt

		a10=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a11=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a12=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a10" >> ${result}/final_result/rpe/mean.txt
		echo "$a11" >> ${result}/final_result/rpe/rmse.txt
		echo "$a12" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -vas --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a9} != ${b2} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
model=sitting_xyz
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Vocabulary/LSDvoc.txt Examples/RGB-D/TUM3_Line.yaml ${datasets}${model}  ${datasets}${model}/associations.txt
		a25=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep mean |awk -F " " '{print $2}'`
		a26=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep rmse |awk -F " " '{print $2}'`
		a27=`evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas | grep std |awk -F " " '{print $2}'`
		echo "$a25" >> ${result}/final_result/mean.txt
		echo "$a26" >> ${result}/final_result/rmse.txt
		echo "$a27" >> ${result}/final_result/std.txt

		a28=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep mean |awk -F " " '{print $2}'`
		a29=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep rmse |awk -F " " '{print $2}'`
		a30=`evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -va | grep std |awk -F " " '{print $2}'`
		echo "$a28" >> ${result}/final_result/rpe/mean.txt
		echo "$a29" >> ${result}/final_result/rpe/rmse.txt
		echo "$a30" >> ${result}/final_result/rpe/std.txt

		evo_rpe tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt  -va  --plot_mode xyz --save_plot ${picture}/rpe/${model}.png 

		evo_ape tum ${datasets}${model}/groundtruth.txt ${camera_exp}/CameraTrajectory.txt -vas  --plot_mode xyz --save_plot ${picture}/${model}.png 


		
		
		# 检查是否为空
		if [[ ${a27} != ${b5} ]];then
			b1=${a1}
			cp -rf ${camera_exp}/KeyFrameTrajectory.txt ${result_keyframe}
			mv -f ${result_keyframe}/KeyFrameTrajectory.txt ${result_keyframe}/KeyFrameTrajectory_${model}.txt
			cp -rf ${camera_exp}/CameraTrajectory.txt ${result_camera}
			mv -f ${result_camera}/CameraTrajectory.txt ${result_camera}/CameraTrajectory_${model}.txt
			
		
		fi
break
#---------------------------------------------------------------------------------------
done