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
    _PORT = 5000;
    _API = `http://${document.location.host.split(':')[0]}:${_PORT}/api`;

    /*
        Saves the params state for the given 
    */
    static save(params) {

    }

    static async _fetch(endpoint) {
        let url = `${Backend._API}/${endpoint}`;
        return fetch(url);
    }

    static async fetchLogBlocks() {
        let blocks = [];

        let result = await Backend._fetch('get_log_blocks');

        return blocks;
    }

}