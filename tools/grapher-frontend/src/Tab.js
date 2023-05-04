import { useEffect, useState } from "react";
import AVAILABLE_LOG_TYPES from "./LogTypes";


function SideBarParam({ param, onCheckChange }) {

    const [checked, setChecked] = useState(param.checked);

    const handleChange = () => {
        let new_checked_value = !checked;
        setChecked(new_checked_value);
        onCheckChange(param, new_checked_value);
    };

    return (
        <div className="row">
            <div className="col-6">
                { param.name }
            </div>
            <div className="col-4">
                { param.type }
            </div>
            <div className="col-2">
                <input class="form-check-input"
                       type="checkbox"
                       checked={checked}
                       onChange={handleChange}
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
        <div>
            <h2>{ config.name }</h2>

            <div className="row">
                <div className="col-3">
                    {params.map(param => <SideBarParam param={param} onCheckChange={onCheckChange}/>)}
                </div>
                <div className="col-7">

                </div>
            </div>

        </div>
    );
}

export default Tab;