const BACKEND_URL = "http://localhost:9095/";


class Backend {

    static setParams() {}
    static writeDebug() {}

    static get(endpoint) {
        let url = BACKEND_URL + endpoint;
        fetch(url, {
            method: 'POST',
        })
        .then(res => res.json())
        .then(res => {
            if (res.status == 'OK') {
                Backend.writeDebug('Saved OK');
            } else {
                Backend.writeDebug(`Saved error: ${res.status}`);
            }
        })
        .catch(err => Backend.writeDebug(`Failed to fetch url ${url}: ${err}`));
    }

    static async saveSettings() {
        Backend.writeDebug('Saving settings');
        Backend.get('save_settings');
    }
    static async setParamValue(param_name, param_address, param_value) {
        Backend.writeDebug(`Setting parameter ${param_name} at ${param_address} to ${param_value}`);
    }
    static loadFile() {

    }

    static readFile(e) {
        let file = e.target.files[0];
        if (!file) {
            return;
        }

        let file_reader = new FileReader();
        file_reader.onload = e => {
            let json = JSON.parse(e.target.result);
            console.log(json)
            Backend.setParams(json);
        };

        Backend.writeDebug(`Reading file ${file.name}`);
        file_reader.readAsText(file);
    }
}

export default Backend;
