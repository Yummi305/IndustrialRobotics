classdef StoreeStop
   methods (Static)
      function out = setgeteStop(data)
         persistent eStop;
         if nargin
            eStop = data;
         end
         out = eStop;
      end
   end
end