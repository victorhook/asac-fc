import { useEffect, useState } from "react";
import History from "./History";


function DisplayParam({ param }) {
    return (
        <div className="row">
            <span className="col-8">
                {param.name}
            </span>
            <span className="col-4">
                {param.value}
            </span>
        </div>
    )
}

function SideBarParam({ param, onPlotChecked, onDisplayChecked }) {

    const [plotChecked, setplotChecked] = useState(param.plotChecked);
    const [displayChecked, setdisplayChecked] = useState(param.displayChecked);

    const handleChangeDisplay = () => {
        let new_checked_value = !displayChecked;
        setdisplayChecked(new_checked_value);
        onDisplayChecked(param, new_checked_value);
    };

    const handleChangePlot = () => {
        let new_checked_value = !plotChecked;
        setplotChecked(new_checked_value);
        onPlotChecked(param, new_checked_value);
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
                <input className="form-check-input"
                       type="checkbox"
                       checked={displayChecked}
                       onChange={handleChangeDisplay}
                />
            </div>
            <div className="col-2">
                <input className="form-check-input"
                       type="checkbox"
                       checked={plotChecked}
                       onChange={handleChangePlot}
                />
            </div>
        </div>
    )
}

function Tab({ config, params, setParams, displayParams, plotParams, displayHistory, plotHistory }) {

    const getParamByName = param_name => {
        for (let param of params) {
            if (param.name == param_name) {
                return param;
            }
        }
    }

    const onDisplayChecked = (param, checked) => {
        setParams({
            ...params,
            [param.name]: {
                ...params[param.name],
                displayChecked: checked
            }
        });
    }

    const onPlotChecked = (param, checked) => {

    }

    return (
        <div className="mt-3">
            <h2>{ config.name }</h2>

            <div className="row">

                {/* -- Sidebar -- */}
                <div className="col-3">

                    {/* Param Header */}
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

                    {/* Parameters */}
                    {
                        Object.keys(params).map(param_name =>
                            <SideBarParam param={params[param_name]}
                                          key={param_name}
                                          onDisplayChecked={onDisplayChecked}
                                          onPlotChecked={onPlotChecked}
                            />)
                    }
                </div>

                {/* Main tab content */}
                <div className="col-9">

                    <div className="row">

                        {/* Display */}
                        <div className="col-3">
                            <h2 className="text-center">Display</h2>
                            <hr />
                            {
                                Object.keys(displayParams).map(param_name =>
                                        <DisplayParam param={params[param_name]}
                                                      key={param_name}
                                        />)
                            }
                        </div>

                        {/* Plot */}
                        <div className="col-9">
                            <h2 className="text-center">Plot</h2>
                            <hr />
                        </div>
                    </div>

                    {/* History */}
                    <div>
                        <History params={params} displayHistory={displayHistory} />
                    </div>
                </div>
            </div>

        </div>
    );
}

export default Tab;