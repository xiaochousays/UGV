classdef SimBase < handle
    properties
        Width=400;
        Length=800;
        Matrix;
        
    end
    
    
    methods
        function Base=SimBase(Width,Length)
            if(nargin>0)
                Base.Width=Width;
                Base.Length=Length;
            end
            Base.Matrix=zeros(Base.Width,Base.Length);
        end
        
        
        function Obj=ImportMap(Obj,str)
            Obj.Matrix=flipud(imread(str));
            Obj.Width=size(Obj.Matrix,1);
            Obj.Length=size(Obj.Matrix,2);
            Show(Obj);
        end
        
        
        function Base=AddWall(Base,StartPoint,Width,Length)
            Base.Matrix(StartPoint(2):(StartPoint(2)+Length),StartPoint(1):(StartPoint(1)+Width))=1;
        end
        
        
        function Show(Base)
            BaseFigure = figure;
            set(BaseFigure,'name','Figure of the workspace','numbertitle','off')
            imshow(Base.Matrix);
            set(gca, 'Ydir', 'normal');
            set(gca, 'XLim', [1 Base.Length], 'YLim', [1 Base.Width]);
        end
    end
    
    
end