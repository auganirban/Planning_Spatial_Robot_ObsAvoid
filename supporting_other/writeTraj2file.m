function[] = writeTraj2file(file_name, seed_val)
    % Load the data from mat file
    data = load(file_name);
    name_of_textfile = strcat("../files_traj/joint_vec_seed", num2str(seed_val), ".txt");
    
    % Write data to a file
    fileID = fopen(name_of_textfile,'w');
    fprintf(fileID,"%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f\n", data.motion_angle_vec);
    fclose(fileID);
end
