import { useEffect, useState } from "react";
import AVAILABLE_LOG_TYPES from "./LogTypes.js";


function SideBarParam({ param, onCheckChange }) {

    const [displayChecked, setdisplayChecked] = useState(param.checked);
    const [plotChecked, setplotChecked] = useState(param.checked);

    const handleChangeDisplay = () => {
        let new_checked_value = !displayChecked;
        setdisplayChecked(new_checked_value);
        onCheckChange(param, new_checked_value);
    };

    const handleChangePlot = () => {
        let new_checked_value = !plotChecked;
        setplotChecked(new_checked_value);
        onCheckChange(param, new_checked_value);
    };

    return (
        <div className="row">
            <div className="col-6">
                { param.name }
            </div>
            <div className="col-2">
                { param.type }
            </div>
            <div className="col-2">
                <input class="form-check-input"
                       type="checkbox"
                       checked={displayChecked}
                       onChange={handleChangeDisplay}
                />
            </div>
            <div className="col-2">
                <input class="form-check-input"
                       type="checkbox"
                       checked={plotChecked}
                       onChange={handleChangePlot}
                />
            </div>
        </div>
    )
}

function Tab({ config }) {

    const [params, setParams] = useState([]);

    const onCheckChange = (param, checked) => {
        for (let p of params) {
            if (p.name == param.name) {
                p.checked = checked;
            }
        }
        setParams(params);
        //params[param.name].checked = checked;
        console.log(params);
    }


    useEffect(() => {
        let new_params = AVAILABLE_LOG_TYPES.copyWithin(0, AVAILABLE_LOG_TYPES.length);
        for (let param of new_params) {
            param.checked = false;
        }
        setParams(new_params);
    }, []);

    return (
        <div className="mt-3">
            <h2>{ config.name }</h2>

            <div className="row">
                <div className="col-3">

                {/* Header */}
                <div className="row mb-1">
                    <div className="col-6">
                        <span>Name</span>
                    </div>
                    <div className="col-2">
                        <span>Type</span>
                    </div>
                    <div className="col-2">
                        <span>Display</span>
                    </div>
                    <div className="col-2">
                        <span>Plot</span>
                    </div>
                    <hr />
                </div>

                    {params.map(param => <SideBarParam param={param} onCheckChange={onCheckChange}/>)}
                </div>

                <div className="col-7">

                </div>
            </div>

        </div>
    );
}

export default Tab;