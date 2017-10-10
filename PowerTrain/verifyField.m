function inStruct = verifyField(inStruct, fieldString, defaultValue)
% inStruct is the name of the structure or an array of structures to search
% fieldName is the name of the field for which the function searches
pathList = strsplit(fieldString,'.');
if (length(pathList) == 1)
    if(~isfield(inStruct,fieldString))
        inStruct.(pathList{1}) = defaultValue;
    else
        % do nothing, value is already set
    end 
else 
    a = strfind(fieldString,'.');
    if(~isfield(inStruct,pathList{1}))
        inStruct.(pathList{1}) = [];
    end
     inStruct.(pathList{1}) =  verifyField(inStruct.(pathList{1}), fieldString((a(1)+1):end),defaultValue);
end


end