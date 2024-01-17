function [map] = generate_map_from_image(file_path, show_map)
    % Input image of JPG or PNG and then convert to a PGM
    % file_path: path to input image
    % show_map[bool]: show map or not
    RGB = imread(file_path);
    % resize, calculate column
    RGB2 = imresize(RGB, [400 400]);
    I = rgb2gray(RGB2);

    output_filename = strcat(file_path, ".pgm");

    imwrite(I, output_filename)
    image = imread(output_filename);

    imageNorm = double(image)/255; % normalize from 0-255 to 0-1
    imageOccupancy = 1 - imageNorm;
    map = occupancyMap(imageOccupancy,20);

    if show_map
        figure
        imshow(I)
        show(map)
    end
end