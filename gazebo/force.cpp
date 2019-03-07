void sendForce()
{
    //on crée notre service client pour communiquer avec gazebo
          ros::service::waitForService("/gazebo/apply_body_wrench");

          applyBodyWrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

   //on définit ici les caractéristiques de notre body wrench

          body_name = "base_link";//myDrone ?//nom du robot ou nom du link?
          applyBodyWrench.request.body_name = body_name; //je sais pas si le request est nécessaire

          reference_frame = "base_link";
          applyBodyWrench.request.reference_frame =reference_frame;

          //referencePoint.x=0;
          //referencePoint.y=0;
          //referencePoint.z=0;
          //applyBodyWrench.reference_point = referencePoint;

            force.x=0;
            force.y=0;
            force.z=100;

            wrench.force=force;
            applyBodyWrench.request.wrench=wrench;

          applyBodyWrenchClient.call(applyBodyWrench);
}


