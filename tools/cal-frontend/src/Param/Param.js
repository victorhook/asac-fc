import './param.css';
import { useState, useEffect } from 'react';


function Param({ param }) {

    const [paramValue, setparamValue] = useState(param.initial_value);
    const paramType = param.type == "meas" ? "Measurement" : "Calibration" ;
    const addressHex = '0x' + param.address.toString(16).toUpperCase();

    const onParamChange = change => {
        setparamValue(change.target.value);
    }

    const paramSet = () => {
        console.log('ok', paramValue);
    }

    return <tr className={`param param-${paramType}`}>
        <td className="param-item"><p>{ param.name }</p></td>
        <td className="param-item"><p>{addressHex}</p></td>
        <td className="param-item"><p>{ param.address }</p></td>
        <td className="param-item"><p>{ paramType }</p></td>
        <td className="param-item">
            <input
                type="text"
                value={paramValue}
                onChange={onParamChange}

            />
        </td>
        <td className="param-item">
            <button className="button param-set-button" onClick={paramSet}>Set</button>
        </td>
    </tr>
}

function ParamList({ params }) {
    return <tbody>
          {params.map(param => <Param param={param}/>)}
    </tbody>
}

function ParamHeader() {
    return <thead>
        <tr>
            <td className="param-header-item">Name</td>
            <td className="param-header-item">Address hex</td>
            <td className="param-header-item">Address dec</td>
            <td className="param-header-item">Param Type</td>
            <td className="param-header-item">Value</td>
        </tr>
    </thead>
}

export { ParamList, ParamHeader };