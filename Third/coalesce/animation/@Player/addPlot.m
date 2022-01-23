function addPlot(this)
% add subplot axes into Maikhail's 
    
    
        A = this.scene.mass.A;
        x = A(1,4); y = A(2,4);
        plot( x, y ,'o', 'Parent',  this.handles.addSub);
        
end % run
