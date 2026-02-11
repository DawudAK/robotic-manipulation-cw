function ids = dxl_scan_ids(port_num, protocol)
% Scans IDs 0..252 using ping, prints IDs that respond.

ids = [];
fprintf("Scanning IDs...\n");
for id = 0:252
    model = pingGetModelNum(port_num, protocol, id);
    if model ~= 0
        fprintf("Found ID %3d (Model %d)\n", id, model);
        ids(end+1) = id; %#ok<AGROW>
    end
end
fprintf("Done. Found %d device(s).\n", numel(ids));
end
