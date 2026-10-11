function root = wg_root()
%WG_ROOT Default output root: <repo>/captures/generated.
root = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'captures', 'generated');
end
