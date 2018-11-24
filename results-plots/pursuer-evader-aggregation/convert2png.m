for i=32:33
%     filename = i+'_1.fig';
    sourcefile = strcat(num2str(i),'_1.fig');
    destination = strcat(strcat('../../presentation/images/',num2str(i)),'_1.png');
    saveas(openfig(sourcefile), destination);
end;