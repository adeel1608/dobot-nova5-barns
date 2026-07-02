# GUI assets

Place optional reference images for editable points here. The Streamlit app
looks for an image whose filename matches the params key path:

    GRAB_PAPER_CUP_PARAMS['7oz']['approach'].png
    PAPER_CUPS_NAVIGATION_PARAMS['twist_7oz'].png

If a matching file is found it is rendered next to the point's old/new
values. The lookup is case-sensitive and uses the same `format_key_path`
representation rendered in the UI.

No images ship by default; add them per deployment as needed.
