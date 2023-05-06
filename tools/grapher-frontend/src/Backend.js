let config = {
    blocks: [
        {
            name: "Gyro",
            params: [
                "raw_gyro_x",
                "raw_gyro_y",
                "raw_gyro_z",
            ]
        }
    ]
};


class Backend {
    static _PORT = 9090;
    static _API = `http://${document.location.host.split(':')[0]}:${Backend._PORT}/api`;

    /*
        Saves the params state for the given 
    */
    static save(params) {

    }

    static async _fetch(endpoint) {
        let url = `${Backend._API}/${endpoint}`;
        //console.log(`Fetching: ${url}`);
        return fetch(url);
    }

    static async fetchLogBlocks() {
        let blocks;
        try {
            let result = await Backend._fetch('get_log_blocks');
            blocks = await result.json();
        } catch(error) {
            blocks = [];
            console.log('Error fetching log blocks!');
        }
        return blocks;
    }

}

export default Backend;
