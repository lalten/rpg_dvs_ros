function distortion ()

cp = cameraParameters( ...
    'IntrinsicMatrix', [164.0138, 0, 80.1241; 0, 164.8755, 44.2025; 0, 0, 1], ...
    'RadialDistortion', [-0.2875, 0.1980], ...
    'TangentialDistortion', [-0.0038, 0.0081]);

[X,Y]=meshgrid(linspace(-63.5,63.5,20));

xy = [X(:),Y(:)];

unXY = undistortPoints(xy, cp);

uv = unXY - xy;

%xy = xy + 63.5;

quiver(xy(:,1),xy(:,2),uv(:,1),uv(:,2));
axis tight equal;

end