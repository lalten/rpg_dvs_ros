% Porno-Plot some pattern data
% get fieldimage_points0x etc from uiimport()ing the csv data file

figure(1)
filename = 'patterns.gif';

ColorSet = varycolor(round((length(time)-2)/4));
set(gca, 'ColorOrder', ColorSet);

axis tight equal;
ylim([0 127]);
xlim([0 127]);

title('Pattern recognitions (left)');
xlabel('X');
ylabel('Y');
zlabel('t');

hold all;
for i=2:length(time)
    points = [ ...
        fieldimage_points0x(i), fieldimage_points0y(i);
        fieldimage_points1x(i), fieldimage_points1y(i);
        fieldimage_points2x(i), fieldimage_points2y(i);
        fieldimage_points3x(i), fieldimage_points3y(i);
        fieldimage_points4x(i), fieldimage_points4y(i);
        fieldimage_points5x(i), fieldimage_points5y(i);
        fieldimage_points6x(i), fieldimage_points6y(i);
        fieldimage_points7x(i), fieldimage_points7y(i);
        fieldimage_points8x(i), fieldimage_points8y(i);
        fieldimage_points9x(i), fieldimage_points9y(i);
        fieldimage_points10x(i), fieldimage_points10y(i);
        fieldimage_points11x(i), fieldimage_points11y(i);
        ];
        
    plot3(points(:,1), points(:,2), repmat(i,[12,1]), '-o');
    
    view([i, 45]);
    drawnow;
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i == 2;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
    
    i

end

hold off;

