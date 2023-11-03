classdef StoreSwitchButtons
   methods (Static)
      function out = setgeteStop(data)
         persistent eStop;
         if nargin
            eStop = data;
         end
         out = eStop;
      end

      function out = setgetManual(data)
         persistent manual;
         if nargin
            manual = data;
         end
         out = manual;
      end

      function out = setgetCow(data)
         persistent MooCow;
         if nargin
            MooCow = data;
         end
         out = MooCow;
      end
   end
end